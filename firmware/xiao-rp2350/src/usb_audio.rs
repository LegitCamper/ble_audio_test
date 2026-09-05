//! Descriptor-driven USB Audio Class 1/2 playback host.

use embassy_usb::handlers::EnumerationInfo;
use embassy_usb::host::descriptor::EndpointDescriptor;
use embassy_usb_driver::host::{ChannelError, HostError, RequestType, SetupPacket, UsbChannel, UsbHostDriver, channel};
use embassy_usb_driver::{EndpointInfo, EndpointType};

use crate::iso::IsoEndpoint;
use ble_audio_uac_descriptor::{
    AudioClassVersion, AudioEndpoint, BYTES_PER_FRAME, FeatureUnitVolume, PlaybackConfig, find_playback,
};

const CS_SAM_FREQ_CONTROL: u8 = 0x01;
const FU_VOLUME_CONTROL: u8 = 0x02;
const REQUEST_CUR: u8 = 0x01;
const REQUEST_RANGE: u8 = 0x02;
// UAC1 encodes transfer direction in the high bit of `bRequest` as well as in `bmRequestType`, so
// a read uses a different request code than the UAC2 equivalent.
const UAC1_REQUEST_GET_CUR: u8 = 0x81;
const UAC1_REQUEST_GET_MIN: u8 = 0x82;
const UAC1_REQUEST_GET_MAX: u8 = 0x83;
/// Channel 0 addresses a feature unit's master control.
const VOLUME_CHANNEL_MASTER: u8 = 0;
/// Full-scale gain in Q15, applied when the host owns volume and nothing is attenuating.
const UNITY_GAIN_Q15: u16 = 1 << 15;
/// UAC reserves this volume code for silence rather than treating it as a real level.
const VOLUME_SILENCE: i16 = i16::MIN;
/// Highest level on the LE Audio volume scale.
const MAXIMUM_LEVEL: u16 = 255;
const DESCRIPTOR_BUFFER_SIZE: usize = 1_024;

/// Playback rate requested from the DAC, matching the decoded LC3 stream.
///
/// This is the same rate [`find_playback`] screens descriptors against, so a selected alternate is
/// always one this host can actually drive.
pub const SAMPLE_RATE_HZ: u32 = ble_audio_uac_descriptor::SAMPLE_RATE_HZ;
/// Stereo frames in one nominal 1 ms Full-Speed packet at [`SAMPLE_RATE_HZ`].
pub const NOMINAL_PACKET_FRAMES: u32 = SAMPLE_RATE_HZ / 1_000;
/// Feedback may pull each packet this far either side of [`NOMINAL_PACKET_FRAMES`].
const PACKET_FRAME_SLACK: u32 = 4;
/// Smallest packet the feedback loop may request.
pub const MINIMUM_PACKET_FRAMES: u32 = NOMINAL_PACKET_FRAMES - PACKET_FRAME_SLACK;
/// Largest packet the feedback loop may request.
pub const MAXIMUM_PACKET_FRAMES: u32 = NOMINAL_PACKET_FRAMES + PACKET_FRAME_SLACK;

/// Which side of the bridge applies volume.
///
/// Exactly one side ever does. The choice is made once at configure time and never changes for
/// the life of the connection, so attenuation can never be applied twice.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VolumeOwner {
    /// The DAC's feature unit answered a volume probe and applies volume itself.
    Dac,
    /// No usable device control, so the RP2350 scales PCM before the USB write.
    Host,
}

/// Volume state for whichever side owns it.
#[derive(Clone, Copy)]
enum Volume {
    /// The device's feature unit, with the dB range it reported in 1/256 dB units.
    Dac {
        unit: FeatureUnitVolume,
        minimum_db_256: i16,
        maximum_db_256: i16,
    },
    /// Q15 gain applied to each sample on the way into USB DPRAM.
    Host { gain_q15: u16 },
}

/// Configured USB Audio Class playback channels.
pub struct UsbAudioPlayback<H: UsbHostDriver> {
    output: IsoEndpoint,
    feedback: Option<IsoEndpoint>,
    control: H::Channel<channel::Control, channel::InOut>,
    feedback_interval_frames: u32,
    maximum_packet_frames: u8,
    volume: Volume,
}

impl<H: UsbHostDriver> UsbAudioPlayback<H> {
    /// Discovers and configures a 48 kHz, signed 16-bit stereo UAC1 or UAC2 playback path.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] when the device has no compatible playback alternate, cannot run at
    /// [`SAMPLE_RATE_HZ`], or a USB control/channel operation fails.
    pub async fn configure(bus: &H, enumeration: &EnumerationInfo) -> Result<Self, HostError> {
        if enumeration.speed != embassy_usb_driver::Speed::Full {
            return Err(HostError::Other("USB Audio playback requires Full Speed"));
        }

        let endpoint_zero = EndpointInfo::new(
            0.into(),
            EndpointType::Control,
            u16::from(enumeration.device_desc.max_packet_size0).min(enumeration.speed.max_packet_size()),
        );
        let mut control = bus.alloc_channel::<channel::Control, channel::InOut>(
            enumeration.device_address,
            &endpoint_zero,
            enumeration.ls_over_fs,
        )?;

        let mut descriptor_buffer = [0_u8; DESCRIPTOR_BUFFER_SIZE];
        let configuration = enumeration
            .active_config_or_set_default(&mut control, &mut descriptor_buffer)
            .await?;
        let selection = find_playback(configuration.buffer).ok_or(HostError::Other(
            "48 kHz stereo S16 USB Audio playback interface missing",
        ))?;
        configure_sample_rate(&mut control, selection).await?;
        set_interface(&mut control, selection.interface_number, selection.alternate_setting).await?;

        let output = IsoEndpoint::new(enumeration.device_address, endpoint_descriptor(selection.output))?;
        let feedback = match selection.feedback {
            Some(endpoint) => Some(IsoEndpoint::new(
                enumeration.device_address,
                endpoint_descriptor(endpoint),
            )?),
            None => None,
        };
        let maximum_packet_frames = (output.max_packet_size() / BYTES_PER_FRAME).min(MAXIMUM_PACKET_FRAMES as usize);
        if maximum_packet_frames < NOMINAL_PACKET_FRAMES as usize {
            return Err(HostError::Other("USB Audio endpoint cannot carry 48 kHz stereo"));
        }

        defmt::info!(
            "USB Audio configured: vid={=u16:04x} pid={=u16:04x} uac={} interface={} alt={} output_ep={=u8:02x} feedback_ep={=u8:02x}",
            enumeration.device_desc.vendor_id,
            enumeration.device_desc.product_id,
            match selection.version {
                AudioClassVersion::Uac1 => 1,
                AudioClassVersion::Uac2 => 2,
            },
            selection.interface_number,
            selection.alternate_setting,
            selection.output.address,
            selection.feedback.map_or(0, |endpoint| endpoint.address),
        );

        // Probe before committing: a device can advertise a feature unit whose volume control
        // stalls in practice, and discovering that mid-stream would leave nothing applying
        // volume at all.
        let volume = match selection.volume {
            Some(unit) => match probe_device_volume(&mut control, selection.version, unit).await {
                Some((minimum_db_256, maximum_db_256)) => {
                    defmt::info!(
                        "DAC owns volume: unit={} master={} range={}..={} (1/256 dB)",
                        unit.unit_id,
                        unit.master,
                        minimum_db_256,
                        maximum_db_256
                    );
                    Volume::Dac {
                        unit,
                        minimum_db_256,
                        maximum_db_256,
                    }
                }
                None => {
                    defmt::info!("DAC advertised volume but failed the probe; RP2350 owns volume");
                    Volume::Host {
                        gain_q15: UNITY_GAIN_Q15,
                    }
                }
            },
            None => {
                defmt::info!("DAC exposes no volume control; RP2350 owns volume");
                Volume::Host {
                    gain_q15: UNITY_GAIN_Q15,
                }
            }
        };

        Ok(Self {
            output,
            feedback,
            control,
            feedback_interval_frames: selection
                .feedback
                .map_or(0, |endpoint| full_speed_interval(endpoint.interval)),
            maximum_packet_frames: u8::try_from(maximum_packet_frames)
                .map_err(|_| HostError::Other("invalid USB Audio packet capacity"))?,
            volume,
        })
    }

    /// Fills USB DPRAM in place and sends one Full-Speed USB audio packet.
    ///
    /// This avoids an intermediate packet buffer; the final copy into the controller's dedicated
    /// DPRAM remains mandatory on RP2350.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] if the isochronous transaction fails.
    pub fn write_packet_with(&mut self, packet_len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), HostError> {
        // Read the gain out before borrowing the endpoint, and skip the pass entirely at unity so
        // a DAC-owned or full-volume stream costs nothing.
        let gain_q15 = match self.volume {
            Volume::Host { gain_q15 } if gain_q15 != UNITY_GAIN_Q15 => Some(gain_q15),
            Volume::Host { .. } | Volume::Dac { .. } => None,
        };
        self.output.write_with(packet_len, |packet| {
            fill(packet);
            if let Some(gain_q15) = gain_q15 {
                attenuate(packet, gain_q15);
            }
        })
    }

    /// Reports which side of the bridge applies volume, decided once at configure time.
    #[must_use]
    pub const fn volume_owner(&self) -> VolumeOwner {
        match self.volume {
            Volume::Dac { .. } => VolumeOwner::Dac,
            Volume::Host { .. } => VolumeOwner::Host,
        }
    }

    /// Applies a volume setting on the LE Audio scale, where `level` is 0..=255.
    ///
    /// When the DAC owns volume this issues a feature unit request; otherwise it updates the gain
    /// applied to subsequent packets, which cannot fail.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] if a device-owned volume request fails. The caller may keep playing;
    /// the stream stays at whatever the device last accepted.
    pub async fn set_volume(&mut self, level: u8, muted: bool) -> Result<(), HostError> {
        match self.volume {
            Volume::Dac {
                unit,
                minimum_db_256,
                maximum_db_256,
            } => {
                let setting = if muted {
                    VOLUME_SILENCE
                } else {
                    device_volume_setting(level, minimum_db_256, maximum_db_256)
                };
                set_device_volume(&mut self.control, unit, setting).await
            }
            Volume::Host { .. } => {
                self.volume = Volume::Host {
                    gain_q15: if muted { 0 } else { host_gain_q15(level) },
                };
                Ok(())
            }
        }
    }

    /// Reads the DAC's explicit feedback value as 16.16 samples per USB frame.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] if the transaction fails or returns neither the UAC2 four-byte 16.16
    /// form nor the three-byte 10.14 form used by some Full-Speed devices.
    pub fn read_feedback(&mut self) -> Result<u32, HostError> {
        let feedback = self
            .feedback
            .as_ref()
            .ok_or(HostError::Other("USB Audio interface has no feedback endpoint"))?;
        let mut bytes = [0_u8; 4];
        let count = feedback.read(&mut bytes)?;
        match count {
            4 => Ok(u32::from_le_bytes(bytes)),
            3 => {
                let q10_14 = u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16);
                Ok(q10_14 << 2)
            }
            _ => Err(ChannelError::BadResponse.into()),
        }
    }

    /// Number of Full-Speed frames between feedback transactions.
    #[must_use]
    pub const fn feedback_interval_frames(&self) -> u32 {
        self.feedback_interval_frames
    }

    /// Largest stereo frame count one Full-Speed packet may carry, after clamping the
    /// endpoint's capacity to [`MAXIMUM_PACKET_FRAMES`].
    #[must_use]
    pub const fn maximum_packet_frames(&self) -> u8 {
        self.maximum_packet_frames
    }

    /// Waits for the next USB Start-of-Frame boundary.
    pub async fn wait_for_sof(&self) {
        self.output.wait_for_next_sof().await;
    }
}

/// Scales each signed 16-bit sample in place by a Q15 gain.
///
/// A trailing odd byte cannot occur because packets are whole stereo frames, and is ignored
/// rather than panicking if one ever does.
fn attenuate(packet: &mut [u8], gain_q15: u16) {
    for sample in packet.chunks_exact_mut(2) {
        let value = i32::from(i16::from_le_bytes([sample[0], sample[1]]));
        // `gain_q15` never exceeds unity, so the product always fits back into `i16`.
        let scaled = (value * i32::from(gain_q15)) >> 15;
        sample.copy_from_slice(&(scaled as i16).to_le_bytes());
    }
}

/// Maps an LE Audio level onto a Q15 gain with a squared taper, which tracks perceived loudness
/// far better than scaling the level linearly.
fn host_gain_q15(level: u8) -> u16 {
    let level = u32::from(level);
    let maximum = u32::from(MAXIMUM_LEVEL);
    ((level * level * u32::from(UNITY_GAIN_Q15)) / (maximum * maximum)) as u16
}

/// Maps an LE Audio level onto a device volume in 1/256 dB, clamped to the range the device
/// reported.
fn device_volume_setting(level: u8, minimum_db_256: i16, maximum_db_256: i16) -> i16 {
    if level == 0 {
        return VOLUME_SILENCE;
    }
    let span = i32::from(maximum_db_256) - i32::from(minimum_db_256);
    // The same squared taper as the host path, so switching owners does not change the feel of
    // the control.
    let level = i32::from(level);
    let maximum = i32::from(MAXIMUM_LEVEL);
    let scaled = (span * level * level) / (maximum * maximum);
    (i32::from(minimum_db_256) + scaled).clamp(i32::from(minimum_db_256), i32::from(maximum_db_256)) as i16
}

const fn volume_control_value(channel: u8) -> u16 {
    (FU_VOLUME_CONTROL as u16) << 8 | channel as u16
}

const fn feature_unit_index(unit: FeatureUnitVolume) -> u16 {
    (unit.unit_id as u16) << 8 | unit.control_interface as u16
}

/// Reads a feature unit's usable volume range, returning `None` if the device does not actually
/// answer the request it advertised.
async fn probe_device_volume<D, C>(
    control: &mut C,
    version: AudioClassVersion,
    unit: FeatureUnitVolume,
) -> Option<(i16, i16)>
where
    D: channel::IsIn + channel::IsOut,
    C: UsbChannel<channel::Control, D>,
{
    let channel = if unit.master { VOLUME_CHANNEL_MASTER } else { 1 };
    let range = match version {
        AudioClassVersion::Uac2 => read_uac2_volume_range(control, unit, channel).await,
        AudioClassVersion::Uac1 => read_uac1_volume_range(control, unit, channel).await,
    };
    let (minimum, maximum) = match range {
        Ok(range) => range,
        Err(error) => {
            defmt::warn!("volume probe failed: {:?}", error);
            return None;
        }
    };
    // A device that reports an empty or inverted range gives nothing to interpolate over.
    if minimum >= maximum || minimum == VOLUME_SILENCE && maximum == VOLUME_SILENCE {
        defmt::warn!("volume probe returned an unusable range {}..={}", minimum, maximum);
        return None;
    }
    Some((minimum, maximum))
}

/// Reads the UAC2 Volume Control `RANGE` payload, which is a subrange count followed by
/// `(MIN, MAX, RES)` triples of signed 1/256 dB values.
async fn read_uac2_volume_range<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    unit: FeatureUnitVolume,
    channel: u8,
) -> Result<(i16, i16), HostError> {
    let mut payload = [0_u8; 2 + 8 * 6];
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_RANGE,
        value: volume_control_value(channel),
        index: feature_unit_index(unit),
        length: payload.len() as u16,
    };
    let count = control.control_in(&request, &mut payload).await?;
    if count < 2 {
        return Err(ChannelError::BadResponse.into());
    }
    let subranges = usize::from(u16::from_le_bytes([payload[0], payload[1]]));
    if subranges == 0 || 2 + subranges * 6 > count {
        return Err(ChannelError::BadResponse.into());
    }
    // Span every subrange, so a device that splits its scale still gets a usable range.
    let mut minimum = i16::MAX;
    let mut maximum = i16::MIN;
    for subrange in payload[2..2 + subranges * 6].chunks_exact(6) {
        minimum = minimum.min(i16::from_le_bytes([subrange[0], subrange[1]]));
        maximum = maximum.max(i16::from_le_bytes([subrange[2], subrange[3]]));
    }
    Ok((minimum, maximum))
}

/// Reads UAC1 `GET_MIN` and `GET_MAX` for a feature unit's volume control.
async fn read_uac1_volume_range<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    unit: FeatureUnitVolume,
    channel: u8,
) -> Result<(i16, i16), HostError> {
    let minimum = read_uac1_volume(control, unit, channel, UAC1_REQUEST_GET_MIN).await?;
    let maximum = read_uac1_volume(control, unit, channel, UAC1_REQUEST_GET_MAX).await?;
    Ok((minimum, maximum))
}

async fn read_uac1_volume<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    unit: FeatureUnitVolume,
    channel: u8,
    request: u8,
) -> Result<i16, HostError> {
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request,
        value: volume_control_value(channel),
        index: feature_unit_index(unit),
        length: 2,
    };
    let mut bytes = [0_u8; 2];
    let count = control.control_in(&request, &mut bytes).await?;
    if count != bytes.len() {
        return Err(ChannelError::BadResponse.into());
    }
    Ok(i16::from_le_bytes(bytes))
}

/// Writes a volume setting to the feature unit, covering both channels when the unit has no
/// master control.
async fn set_device_volume<D, C>(control: &mut C, unit: FeatureUnitVolume, setting: i16) -> Result<(), HostError>
where
    D: channel::IsIn + channel::IsOut,
    C: UsbChannel<channel::Control, D>,
{
    // `SET_CUR` is request 0x01 in both UAC1 and UAC2.
    if unit.master {
        return write_volume_channel(control, unit, VOLUME_CHANNEL_MASTER, setting).await;
    }
    write_volume_channel(control, unit, 1, setting).await?;
    write_volume_channel(control, unit, 2, setting).await
}

async fn write_volume_channel<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    unit: FeatureUnitVolume,
    channel: u8,
    setting: i16,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: volume_control_value(channel),
        index: feature_unit_index(unit),
        length: 2,
    };
    control.control_out(&request, &setting.to_le_bytes()).await?;
    Ok(())
}

fn endpoint_descriptor(endpoint: AudioEndpoint) -> EndpointDescriptor {
    EndpointDescriptor {
        len: 7,
        descriptor_type: 5,
        endpoint_address: endpoint.address,
        attributes: endpoint.attributes,
        max_packet_size: endpoint.max_packet_size,
        interval: endpoint.interval,
    }
}

async fn configure_sample_rate<D, C>(control: &mut C, selection: PlaybackConfig) -> Result<(), HostError>
where
    D: channel::IsIn + channel::IsOut,
    C: UsbChannel<channel::Control, D>,
{
    match selection.version {
        AudioClassVersion::Uac1 => {
            if selection.uac1_sample_rate_control {
                set_uac1_endpoint_frequency(control, selection.output.address, SAMPLE_RATE_HZ).await?;
                match get_uac1_endpoint_frequency(control, selection.output.address).await {
                    Ok(SAMPLE_RATE_HZ) => {}
                    Ok(active_rate) => {
                        defmt::warn!("UAC1 endpoint stayed at {} Hz", active_rate);
                        return Err(HostError::Other("UAC1 endpoint did not switch to 48 kHz"));
                    }
                    Err(error) => {
                        defmt::warn!("UAC1 sample-rate verification failed: {:?}", error);
                    }
                }
            }
        }
        AudioClassVersion::Uac2 => {
            let index = clock_control_index(selection.uac2_clock_source, selection.uac2_control_interface);
            let active_rate = get_uac2_clock_frequency(control, index).await?;
            if active_rate == SAMPLE_RATE_HZ {
                return Ok(());
            }
            if !selection.uac2_clock_writable {
                return Err(HostError::Other("read-only UAC2 clock is not 48 kHz"));
            }
            match uac2_clock_range_contains(control, index, SAMPLE_RATE_HZ).await {
                Ok(true) => {}
                Ok(false) => return Err(HostError::Other("UAC2 clock does not support 48 kHz")),
                Err(error) => defmt::warn!("UAC2 clock RANGE query failed: {:?}; trying SET", error),
            }
            set_uac2_clock_frequency(control, index, SAMPLE_RATE_HZ).await?;
            let active_rate = get_uac2_clock_frequency(control, index).await?;
            if active_rate != SAMPLE_RATE_HZ {
                defmt::warn!("UAC2 clock stayed at {} Hz", active_rate);
                return Err(HostError::Other("UAC2 clock did not switch to 48 kHz"));
            }
        }
    }
    Ok(())
}

const fn clock_control_index(clock_source: u8, control_interface: u8) -> u16 {
    (clock_source as u16) << 8 | control_interface as u16
}

async fn set_uac2_clock_frequency<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    index: u16,
    frequency_hz: u32,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index,
        length: 4,
    };
    control.control_out(&request, &frequency_hz.to_le_bytes()).await?;
    Ok(())
}

async fn get_uac2_clock_frequency<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    index: u16,
) -> Result<u32, HostError> {
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index,
        length: 4,
    };
    let mut bytes = [0_u8; 4];
    let count = control.control_in(&request, &mut bytes).await?;
    if count != bytes.len() {
        return Err(ChannelError::BadResponse.into());
    }
    Ok(u32::from_le_bytes(bytes))
}

/// Reports whether the clock entity's Sampling Frequency Control covers `frequency_hz`.
///
/// The UAC2 RANGE payload is a subrange count followed by `(dMIN, dMAX, dRES)` triples. A zero
/// resolution means the subrange is continuous.
async fn uac2_clock_range_contains<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    index: u16,
    frequency_hz: u32,
) -> Result<bool, HostError> {
    // 14 subranges is far more than any Full-Speed DAC advertises, and keeps this off the stack
    // budget of the USB task.
    let mut payload = [0_u8; 2 + 14 * 12];
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_RANGE,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index,
        length: payload.len() as u16,
    };
    let count = control.control_in(&request, &mut payload).await?;
    if count < 2 {
        return Err(ChannelError::BadResponse.into());
    }
    let subranges = usize::from(u16::from_le_bytes([payload[0], payload[1]]));
    if subranges == 0 || 2 + subranges * 12 > count {
        return Err(ChannelError::BadResponse.into());
    }

    for subrange in payload[2..2 + subranges * 12].chunks_exact(12) {
        let minimum = u32::from_le_bytes([subrange[0], subrange[1], subrange[2], subrange[3]]);
        let maximum = u32::from_le_bytes([subrange[4], subrange[5], subrange[6], subrange[7]]);
        let resolution = u32::from_le_bytes([subrange[8], subrange[9], subrange[10], subrange[11]]);
        defmt::info!("DAC clock subrange {}..={} step {}", minimum, maximum, resolution);
        if (minimum..=maximum).contains(&frequency_hz)
            && (resolution == 0 || (frequency_hz - minimum) % resolution == 0)
        {
            return Ok(true);
        }
    }
    Ok(false)
}

async fn set_uac1_endpoint_frequency<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    endpoint_address: u8,
    frequency_hz: u32,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_CLASS | RequestType::RECIPIENT_ENDPOINT,
        request: REQUEST_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index: u16::from(endpoint_address),
        length: 3,
    };
    let bytes = frequency_hz.to_le_bytes();
    control.control_out(&request, &bytes[..3]).await?;
    Ok(())
}

async fn get_uac1_endpoint_frequency<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    endpoint_address: u8,
) -> Result<u32, HostError> {
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_ENDPOINT,
        request: UAC1_REQUEST_GET_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index: u16::from(endpoint_address),
        length: 3,
    };
    let mut bytes = [0_u8; 3];
    let count = control.control_in(&request, &mut bytes).await?;
    if count != bytes.len() {
        return Err(ChannelError::BadResponse.into());
    }
    Ok(u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16))
}

async fn set_interface<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    interface: u8,
    alternate_setting: u8,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_STANDARD | RequestType::RECIPIENT_INTERFACE,
        request: 0x0b,
        value: u16::from(alternate_setting),
        index: u16::from(interface),
        length: 0,
    };
    control.control_out(&request, &[]).await?;
    Ok(())
}

const fn full_speed_interval(b_interval: u8) -> u32 {
    if b_interval == 0 || b_interval > 16 {
        return 1;
    }
    1 << (b_interval - 1)
}
