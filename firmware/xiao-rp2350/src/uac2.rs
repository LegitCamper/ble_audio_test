//! Minimal UAC2 playback host class for the TTGK TE-C DAC.

use embassy_usb::handlers::EnumerationInfo;
use embassy_usb::host::descriptor::InterfaceDescriptor;
use embassy_usb_driver::host::{ChannelError, HostError, RequestType, SetupPacket, UsbChannel, UsbHostDriver, channel};
use embassy_usb_driver::{Direction, EndpointInfo, EndpointType};

use crate::iso::IsoEndpoint;

const AUDIO_CLASS: u8 = 1;
const AUDIO_STREAMING_SUBCLASS: u8 = 2;
const UAC2_PROTOCOL: u8 = 0x20;
const PLAYBACK_INTERFACE: u8 = 2;
const PLAYBACK_ALT_16_BIT: u8 = 1;
const CLOCK_SOURCE_ENTITY: u8 = 1;
const PLAYBACK_FEATURE_UNIT: u8 = 10;
const AUDIO_CONTROL_INTERFACE: u8 = 1;
const CS_SAM_FREQ_CONTROL: u8 = 0x01;
const REQUEST_CUR: u8 = 0x01;
const REQUEST_RANGE: u8 = 0x02;

/// Playback rate requested from the DAC, matching the decoded LC3 stream.
pub const SAMPLE_RATE_HZ: u32 = 48_000;
/// Stereo frames in one nominal 1 ms Full-Speed packet at [`SAMPLE_RATE_HZ`].
pub const NOMINAL_PACKET_FRAMES: u32 = SAMPLE_RATE_HZ / 1_000;
/// Feedback may pull each packet this far either side of [`NOMINAL_PACKET_FRAMES`].
const PACKET_FRAME_SLACK: u32 = 4;
/// Smallest packet the feedback loop may request.
pub const MINIMUM_PACKET_FRAMES: u32 = NOMINAL_PACKET_FRAMES - PACKET_FRAME_SLACK;
/// Largest packet the feedback loop may request.
pub const MAXIMUM_PACKET_FRAMES: u32 = NOMINAL_PACKET_FRAMES + PACKET_FRAME_SLACK;

/// Configured UAC2 asynchronous playback channels for the TE-C.
pub struct Uac2Playback<H: UsbHostDriver> {
    output: IsoEndpoint,
    feedback: IsoEndpoint,
    _control: H::Channel<channel::Control, channel::InOut>,
    feedback_interval_frames: u32,
    maximum_packet_frames: u8,
}

impl<H: UsbHostDriver> Uac2Playback<H> {
    /// Configures 48 kHz, 16-bit stereo playback on interface 2, alternate setting 1.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] when descriptors do not match the verified TE-C layout, when the
    /// DAC's clock entity will not run at [`SAMPLE_RATE_HZ`], or when a USB control/channel
    /// operation fails.
    pub async fn configure(bus: &H, enumeration: &EnumerationInfo) -> Result<Self, HostError> {
        if enumeration.speed != embassy_usb_driver::Speed::Full {
            return Err(HostError::Other("TE-C did not enumerate at Full Speed"));
        }
        if enumeration.device_desc.vendor_id != 0x3302 || enumeration.device_desc.product_id != 0x43e8 {
            return Err(HostError::Other("connected device is not TTGK TE-C"));
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

        let mut descriptor_buffer = [0_u8; 512];
        let configuration = enumeration
            .active_config_or_set_default(&mut control, &mut descriptor_buffer)
            .await?;
        let interface = configuration
            .iter_interface()
            .find(is_supported_playback_interface)
            .ok_or(HostError::Other("16-bit UAC2 playback interface missing"))?;
        if !has_pcm_s16_stereo_format(&interface) {
            return Err(HostError::Other("unexpected UAC2 format descriptors"));
        }

        let output_endpoint = interface
            .iter_endpoints()
            .find(|endpoint| {
                endpoint.ep_type() == EndpointType::Isochronous
                    && endpoint.ep_dir() == Direction::Out
                    && endpoint.attributes & 0x30 == 0
            })
            .ok_or(HostError::Other("UAC2 playback endpoint missing"))?;
        let feedback_endpoint = interface
            .iter_endpoints()
            .find(|endpoint| {
                endpoint.ep_type() == EndpointType::Isochronous
                    && endpoint.ep_dir() == Direction::In
                    && endpoint.attributes & 0x30 == 0x10
            })
            .ok_or(HostError::Other("UAC2 feedback endpoint missing"))?;

        // UAC2 only lists supported rates through the clock entity's controls, not in the
        // streaming descriptors, so 48 kHz support has to be asked for rather than parsed. A
        // device that stalls RANGE is tolerated; one that answers and omits 48 kHz, or that
        // accepts the SET and keeps running at another rate, is rejected instead of being fed
        // audio at the wrong speed.
        match clock_frequency_range_contains(&mut control, SAMPLE_RATE_HZ).await {
            Ok(true) => {}
            Ok(false) => return Err(HostError::Other("DAC clock entity does not support 48 kHz")),
            Err(error) => defmt::warn!("clock frequency RANGE query failed: {:?}; trying SET anyway", error),
        }
        set_clock_frequency(&mut control, SAMPLE_RATE_HZ).await?;
        let active_rate = get_clock_frequency(&mut control).await?;
        defmt::info!("DAC active sample rate={} Hz", active_rate);
        if active_rate != SAMPLE_RATE_HZ {
            defmt::warn!(
                "DAC stayed at {} Hz after requesting {} Hz",
                active_rate,
                SAMPLE_RATE_HZ
            );
            return Err(HostError::Other("DAC did not switch to 48 kHz"));
        }
        set_feature_volume(&mut control, PLAYBACK_FEATURE_UNIT, 1, 0).await?;
        set_feature_volume(&mut control, PLAYBACK_FEATURE_UNIT, 2, 0).await?;
        set_interface(&mut control, PLAYBACK_INTERFACE, PLAYBACK_ALT_16_BIT).await?;

        let output = IsoEndpoint::new(enumeration.device_address, output_endpoint)?;
        let feedback = IsoEndpoint::new(enumeration.device_address, feedback_endpoint)?;
        let maximum_packet_frames = (output.max_packet_size() / 4).min(MAXIMUM_PACKET_FRAMES as usize);
        if maximum_packet_frames <= NOMINAL_PACKET_FRAMES as usize {
            return Err(HostError::Other("UAC2 endpoint cannot carry 48 kHz stereo"));
        }

        Ok(Self {
            output,
            feedback,
            _control: control,
            feedback_interval_frames: full_speed_interval(feedback_endpoint.interval),
            maximum_packet_frames: u8::try_from(maximum_packet_frames)
                .map_err(|_| HostError::Other("invalid UAC2 packet capacity"))?,
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
        self.output.write_with(packet_len, fill)
    }

    /// Reads the DAC's explicit feedback value as 16.16 samples per USB frame.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] if the transaction fails or returns neither the UAC2 four-byte 16.16
    /// form nor the three-byte 10.14 form used by some Full-Speed devices.
    pub fn read_feedback(&mut self) -> Result<u32, HostError> {
        let mut bytes = [0_u8; 4];
        let count = self.feedback.read(&mut bytes)?;
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

fn is_supported_playback_interface(interface: &InterfaceDescriptor<'_>) -> bool {
    interface.interface_number == PLAYBACK_INTERFACE
        && interface.alternate_setting == PLAYBACK_ALT_16_BIT
        && interface.interface_class == AUDIO_CLASS
        && interface.interface_subclass == AUDIO_STREAMING_SUBCLASS
        && interface.interface_protocol == UAC2_PROTOCOL
        && interface.num_endpoints == 2
}

fn has_pcm_s16_stereo_format(interface: &InterfaceDescriptor<'_>) -> bool {
    let mut pcm_stereo = false;
    let mut signed_16 = false;
    for (_, descriptor) in interface.iter_descriptors() {
        if descriptor.len() >= 16 && descriptor[1] == 0x24 && descriptor[2] == 1 {
            let formats = u32::from_le_bytes([descriptor[6], descriptor[7], descriptor[8], descriptor[9]]);
            pcm_stereo = descriptor[5] == 1 && formats & 1 != 0 && descriptor[10] == 2;
        }
        if descriptor.len() >= 6 && descriptor[1] == 0x24 && descriptor[2] == 2 {
            signed_16 = descriptor[3] == 1 && descriptor[4] == 2 && descriptor[5] == 16;
        }
    }
    pcm_stereo && signed_16
}

fn clock_control_index() -> u16 {
    (u16::from(CLOCK_SOURCE_ENTITY) << 8) | u16::from(AUDIO_CONTROL_INTERFACE)
}

async fn set_clock_frequency<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    frequency_hz: u32,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index: clock_control_index(),
        length: 4,
    };
    control.control_out(&request, &frequency_hz.to_le_bytes()).await?;
    Ok(())
}

async fn get_clock_frequency<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
) -> Result<u32, HostError> {
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index: clock_control_index(),
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
async fn clock_frequency_range_contains<D: channel::IsIn, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    frequency_hz: u32,
) -> Result<bool, HostError> {
    // 14 subranges is far more than any Full-Speed DAC advertises, and keeps this off the stack
    // budget of the USB task.
    let mut payload = [0_u8; 2 + 14 * 12];
    let request = SetupPacket {
        request_type: RequestType::IN | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_RANGE,
        value: u16::from(CS_SAM_FREQ_CONTROL) << 8,
        index: clock_control_index(),
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

async fn set_feature_volume<D: channel::IsOut, C: UsbChannel<channel::Control, D>>(
    control: &mut C,
    feature_unit: u8,
    channel: u8,
    volume_db_q8_8: i16,
) -> Result<(), HostError> {
    let request = SetupPacket {
        request_type: RequestType::OUT | RequestType::TYPE_CLASS | RequestType::RECIPIENT_INTERFACE,
        request: REQUEST_CUR,
        value: (0x02_u16 << 8) | u16::from(channel),
        index: (u16::from(feature_unit) << 8) | u16::from(AUDIO_CONTROL_INTERFACE),
        length: 2,
    };
    control.control_out(&request, &volume_db_q8_8.to_le_bytes()).await?;
    Ok(())
}

const fn full_speed_interval(b_interval: u8) -> u32 {
    if b_interval == 0 || b_interval > 16 {
        return 1;
    }
    1 << (b_interval - 1)
}
