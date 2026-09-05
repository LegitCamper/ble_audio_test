#![no_std]
#![deny(missing_docs)]

//! Allocation-free USB Audio playback descriptor selection.
//!
//! [`find_playback`] walks a raw USB configuration descriptor and reports the first
//! class-compliant USB Audio Class 1 or 2 alternate setting able to consume 48 kHz, stereo,
//! signed 16-bit PCM over a Full-Speed isochronous OUT endpoint. Interface numbers, alternate
//! settings, endpoint addresses, and UAC2 clock entities all come from the device rather than
//! from per-device constants, so the caller does not need a vendor/product allowlist.
//!
//! Every descriptor field is bounds-checked. Truncated or self-inconsistent descriptors end the
//! walk instead of panicking, so a malicious or broken device cannot fault the host.

const DESCRIPTOR_INTERFACE: u8 = 0x04;
const DESCRIPTOR_ENDPOINT: u8 = 0x05;
const DESCRIPTOR_CS_INTERFACE: u8 = 0x24;
const DESCRIPTOR_CS_ENDPOINT: u8 = 0x25;
const AUDIO_CLASS: u8 = 0x01;
const AUDIO_CONTROL_SUBCLASS: u8 = 0x01;
const AUDIO_STREAMING_SUBCLASS: u8 = 0x02;
const UAC1_PROTOCOL: u8 = 0x00;
const UAC2_PROTOCOL: u8 = 0x20;
const AS_GENERAL: u8 = 0x01;
const FORMAT_TYPE: u8 = 0x02;
const FORMAT_TYPE_I: u8 = 0x01;
const FORMAT_PCM: u16 = 0x0001;
const CLOCK_SOURCE: u8 = 0x0a;
const FEATURE_UNIT: u8 = 0x06;
/// Bit 1 of a UAC1 feature unit's `bmaControls` byte marks a volume control.
const UAC1_VOLUME_CONTROL: u8 = 0x02;
/// Bits 2..3 of a UAC2 feature unit's `bmaControls` word hold the volume control's access mode,
/// where `0b11` is read/write.
const UAC2_VOLUME_CONTROL_SHIFT: u32 = 2;
const UAC2_CONTROL_READ_WRITE: u32 = 0x03;
const INPUT_TERMINAL: u8 = 0x02;
const ENDPOINT_ISOCHRONOUS: u8 = 0x01;
const ENDPOINT_SYNCHRONIZATION_MASK: u8 = 0x0c;
const ENDPOINT_USAGE_MASK: u8 = 0x30;
const ENDPOINT_DATA: u8 = 0x00;
const ENDPOINT_FEEDBACK: u8 = 0x10;

/// Isochronous endpoint synchronization type meaning the sink runs its own clock and reports
/// drift through an explicit feedback endpoint.
pub const ENDPOINT_ASYNCHRONOUS: u8 = 0x04;

/// Playback rate this selector requires, matching the bridge's decoded LC3 stream.
pub const SAMPLE_RATE_HZ: u32 = 48_000;
/// Channels this selector requires.
pub const CHANNEL_COUNT: u8 = 2;
/// Sample resolution in bits this selector requires.
pub const BIT_RESOLUTION: u8 = 16;
/// Bytes in one 48 kHz stereo signed 16-bit frame.
pub const BYTES_PER_FRAME: usize = 4;
/// Smallest isochronous OUT packet capacity that can carry one nominal 1 ms packet.
pub const MINIMUM_PACKET_BYTES: usize = (SAMPLE_RATE_HZ as usize / 1_000) * BYTES_PER_FRAME;

/// Highest level on the LE Audio Volume Control Service scale.
pub const MAXIMUM_LEVEL: u8 = 255;
/// Full-scale Q15 gain.
pub const UNITY_GAIN_Q15: u16 = 1 << 15;
/// USB Audio reserves this volume code for silence rather than treating it as a real level.
pub const VOLUME_SILENCE: i16 = i16::MIN;
/// Attenuation the bottom of the nonzero volume scale corresponds to.
///
/// A 30 dB range keeps the lower half useful on quiet source material: half scale is about
/// -15 dB. Spanning a DAC's whole reported range puts most of the control below audibility, since
/// DACs commonly report a floor near -127 dB. Level zero remains exact silence.
pub const VOLUME_RANGE_DB: i32 = 30;

/// Q15 gains matching [`VOLUME_RANGE_DB`] at evenly spaced levels from 0 to [`MAXIMUM_LEVEL`].
///
/// A table keeps the host path on the same decibel curve as the device path without needing a
/// power function; sixteen segments hold the interpolation error well under the ~0.4 dB a
/// listener can notice.
const HOST_GAIN_Q15: [u16; 17] = [
    1036, 1286, 1596, 1980, 2457, 3049, 3784, 4696, 5827, 7231, 8973, 11135, 13818, 17147, 21279, 26406, 32768,
];

/// Maps an LE Audio level onto a Q15 gain along the same decibel curve [`device_volume_setting`]
/// uses, so moving volume between the host and the device does not change the feel of the control.
#[must_use]
pub fn host_gain_q15(level: u8) -> u16 {
    const SEGMENTS: u32 = 16;

    if level == 0 {
        return 0;
    }
    let scaled = u32::from(level) * SEGMENTS;
    let segment = usize::try_from(scaled / u32::from(MAXIMUM_LEVEL)).unwrap_or(HOST_GAIN_Q15.len() - 1);
    let Some(&lower) = HOST_GAIN_Q15.get(segment) else {
        return UNITY_GAIN_Q15;
    };
    let Some(&upper) = HOST_GAIN_Q15.get(segment + 1) else {
        return lower;
    };
    // Interpolate linearly between the two nearest table entries.
    let position = scaled % u32::from(MAXIMUM_LEVEL);
    let step = u32::from(upper - lower) * position / u32::from(MAXIMUM_LEVEL);
    u16::try_from(u32::from(lower) + step).unwrap_or(UNITY_GAIN_Q15)
}

/// Maps an LE Audio level onto a device volume in 1/256 dB, clamped to the range the device
/// reported.
///
/// The mapping is linear in decibels. Interpolating across the device's whole reported range, or
/// applying an amplitude-style taper on top of a value that is already logarithmic, leaves the
/// lower two thirds of the scale inaudible.
#[must_use]
pub fn device_volume_setting(level: u8, minimum_db_256: i16, maximum_db_256: i16) -> i16 {
    if level == 0 {
        return VOLUME_SILENCE;
    }
    let maximum = i32::from(maximum_db_256);
    let minimum = i32::from(minimum_db_256);
    // Never reach below the device's own floor, but do not follow it all the way down either.
    let floor = (maximum - VOLUME_RANGE_DB * 256).max(minimum);
    let setting = floor + (maximum - floor) * i32::from(level) / i32::from(MAXIMUM_LEVEL);
    i16::try_from(setting.clamp(minimum, maximum)).unwrap_or(maximum_db_256)
}

/// USB Audio specification generation used by a playback alternate setting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AudioClassVersion {
    /// USB Audio Class 1.0, which declares rates in the streaming format descriptor and sets
    /// them through an endpoint control.
    Uac1,
    /// USB Audio Class 2.0, which declares rates through a clock entity on the audio control
    /// interface.
    Uac2,
}

/// Standard endpoint fields needed by the RP2350 EPX driver.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AudioEndpoint {
    /// `bEndpointAddress`, including the direction bit.
    pub address: u8,
    /// `bmAttributes`, carrying transfer, synchronization, and usage type.
    pub attributes: u8,
    /// `wMaxPacketSize` payload bytes, with the high-bandwidth multiplier bits masked off.
    pub max_packet_size: u16,
    /// `bInterval`, a Full-Speed frame exponent for isochronous endpoints.
    pub interval: u8,
}

impl AudioEndpoint {
    /// Synchronization type bits of `bmAttributes`.
    #[must_use]
    pub const fn synchronization_type(self) -> u8 {
        self.attributes & ENDPOINT_SYNCHRONIZATION_MASK
    }

    /// Reports whether the sink drives its own clock and must be paced by explicit feedback.
    #[must_use]
    pub const fn is_asynchronous(self) -> bool {
        self.synchronization_type() == ENDPOINT_ASYNCHRONOUS
    }
}

/// Writable volume control found on a feature unit in the playback path.
///
/// Its presence in the descriptors only means the device *claims* the control; the host still
/// verifies it at configure time before handing volume over to the device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FeatureUnitVolume {
    /// `bInterfaceNumber` of the audio control interface owning the unit.
    pub control_interface: u8,
    /// `bUnitID` of the feature unit carrying the volume control.
    pub unit_id: u8,
    /// Whether channel 0 (master) carries the control. When false the unit is per-channel and
    /// channels 1 and 2 have to be set individually.
    pub master: bool,
}

/// Descriptor-selected 48 kHz stereo signed 16-bit playback path.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PlaybackConfig {
    /// Audio class generation the selected alternate setting speaks.
    pub version: AudioClassVersion,
    /// `bInterfaceNumber` of the streaming interface to activate.
    pub interface_number: u8,
    /// `bAlternateSetting` that exposes the compatible format.
    pub alternate_setting: u8,
    /// Isochronous OUT endpoint carrying PCM to the sink.
    pub output: AudioEndpoint,
    /// Explicit feedback IN endpoint, present only when the device offers one.
    pub feedback: Option<AudioEndpoint>,
    /// Whether the UAC1 streaming endpoint advertises a writable Sampling Frequency Control.
    pub uac1_sample_rate_control: bool,
    /// `bInterfaceNumber` of the UAC2 audio control interface owning the clock entity.
    pub uac2_control_interface: u8,
    /// `bClockID` of the UAC2 clock source feeding this stream.
    pub uac2_clock_source: u8,
    /// Whether that UAC2 clock source accepts a Sampling Frequency Control write.
    pub uac2_clock_writable: bool,
    /// Feature unit volume control the device advertises for this stream, if any.
    pub volume: Option<FeatureUnitVolume>,
}

/// Finds the first class-compliant Full-Speed playback alternate that can consume 48 kHz stereo
/// signed 16-bit PCM.
///
/// `configuration_body` is the entire configuration descriptor as returned by the device,
/// starting at its own 9-byte header. Returns `None` when no alternate qualifies, including when
/// the descriptor is truncated mid-way.
#[must_use]
pub fn find_playback(configuration_body: &[u8]) -> Option<PlaybackConfig> {
    let mut offset = 0;
    while let Some((interface, next)) = descriptor_at(configuration_body, offset) {
        offset = next;
        if interface.get(1) != Some(&DESCRIPTOR_INTERFACE) || interface.len() < 9 {
            continue;
        }

        let body_end = next_interface_offset(configuration_body, offset);
        let body = &configuration_body[offset..body_end];
        if let Some(config) = playback_from_interface(configuration_body, interface, body) {
            return Some(config);
        }
    }
    None
}

fn playback_from_interface(
    configuration_body: &[u8],
    interface: &[u8],
    interface_body: &[u8],
) -> Option<PlaybackConfig> {
    // Alternate setting zero of a streaming interface is required to have no isochronous
    // bandwidth, so it can never be the one to activate.
    if interface[3] == 0 || interface[5] != AUDIO_CLASS || interface[6] != AUDIO_STREAMING_SUBCLASS {
        return None;
    }

    let version = match interface[7] {
        UAC1_PROTOCOL => AudioClassVersion::Uac1,
        UAC2_PROTOCOL => AudioClassVersion::Uac2,
        _ => return None,
    };
    let terminal_link = compatible_stream_format(version, interface_body)?;

    let output = find_endpoint(interface_body, false, ENDPOINT_DATA)?;
    if output.interval != 1 || usize::from(output.max_packet_size) < MINIMUM_PACKET_BYTES {
        return None;
    }
    let feedback = find_endpoint(interface_body, true, ENDPOINT_FEEDBACK).filter(|endpoint| {
        // Full Speed reports 10.14 in three bytes; some UAC2 devices report 16.16 in four.
        (3..=4).contains(&endpoint.max_packet_size) && (1..=16).contains(&endpoint.interval)
    });
    // An asynchronous sink free-runs on its own clock, so without feedback its packet cadence
    // cannot be tracked and the stream would drift into permanent over- or underflow.
    if output.is_asynchronous() && feedback.is_none() {
        return None;
    }

    let (control_interface, clock_source, clock_writable) = match version {
        AudioClassVersion::Uac1 => (0, 0, false),
        AudioClassVersion::Uac2 => find_uac2_clock(configuration_body, terminal_link)?,
    };

    Some(PlaybackConfig {
        version,
        interface_number: interface[2],
        alternate_setting: interface[3],
        output,
        feedback,
        uac1_sample_rate_control: version == AudioClassVersion::Uac1 && has_uac1_sample_rate_control(interface_body),
        uac2_control_interface: control_interface,
        uac2_clock_source: clock_source,
        uac2_clock_writable: clock_writable,
        volume: find_volume_feature_unit(configuration_body, version, terminal_link),
    })
}

/// Returns the `bTerminalLink` of a streaming interface whose format descriptors declare 48 kHz
/// stereo signed 16-bit PCM.
fn compatible_stream_format(version: AudioClassVersion, descriptors: &[u8]) -> Option<u8> {
    let mut terminal_link = None;
    let mut compatible_format = false;
    let mut offset = 0;
    while let Some((descriptor, next)) = descriptor_at(descriptors, offset) {
        offset = next;
        if descriptor.get(1) != Some(&DESCRIPTOR_CS_INTERFACE) {
            continue;
        }
        match (version, descriptor.get(2).copied()) {
            (AudioClassVersion::Uac1, Some(AS_GENERAL)) if descriptor.len() >= 7 => {
                if u16::from_le_bytes([descriptor[5], descriptor[6]]) == FORMAT_PCM {
                    terminal_link = Some(descriptor[3]);
                }
            }
            (AudioClassVersion::Uac1, Some(FORMAT_TYPE)) if descriptor.len() >= 8 => {
                compatible_format = descriptor[3] == FORMAT_TYPE_I
                    && descriptor[4] == CHANNEL_COUNT
                    && descriptor[5] == u8::try_from(BYTES_PER_FRAME / CHANNEL_COUNT as usize).ok()?
                    && descriptor[6] == BIT_RESOLUTION
                    && uac1_supports_rate(descriptor, SAMPLE_RATE_HZ);
            }
            (AudioClassVersion::Uac2, Some(AS_GENERAL)) if descriptor.len() >= 16 => {
                let formats = u32::from_le_bytes([descriptor[6], descriptor[7], descriptor[8], descriptor[9]]);
                if descriptor[5] == FORMAT_TYPE_I && formats & 1 != 0 && descriptor[10] == CHANNEL_COUNT {
                    terminal_link = Some(descriptor[3]);
                }
            }
            // UAC2 declares rates on the clock entity, so the format descriptor only has to
            // agree on subslot size and resolution.
            (AudioClassVersion::Uac2, Some(FORMAT_TYPE)) if descriptor.len() >= 6 => {
                compatible_format = descriptor[3] == FORMAT_TYPE_I
                    && descriptor[4] == u8::try_from(BYTES_PER_FRAME / CHANNEL_COUNT as usize).ok()?
                    && descriptor[5] == BIT_RESOLUTION;
            }
            _ => {}
        }
    }
    compatible_format
        .then_some(terminal_link?)
        .filter(|terminal| *terminal != 0)
}

/// Checks a UAC1 Type I format descriptor's discrete rate list or continuous rate range.
fn uac1_supports_rate(format: &[u8], rate_hz: u32) -> bool {
    let frequency_count = usize::from(format[7]);
    if frequency_count == 0 {
        return format.len() >= 14 && (read_u24(&format[8..11])..=read_u24(&format[11..14])).contains(&rate_hz);
    }
    let Some(frequencies) = format.get(8..8 + frequency_count * 3) else {
        return false;
    };
    frequencies
        .chunks_exact(3)
        .any(|frequency| read_u24(frequency) == rate_hz)
}

/// Resolves the audio control interface and clock entity that feed a UAC2 streaming interface,
/// reporting whether the clock's frequency is writable.
fn find_uac2_clock(configuration_body: &[u8], terminal_link: u8) -> Option<(u8, u8, bool)> {
    let mut offset = 0;
    while let Some((interface, next)) = descriptor_at(configuration_body, offset) {
        offset = next;
        if interface.get(1) != Some(&DESCRIPTOR_INTERFACE)
            || interface.len() < 9
            || interface[5] != AUDIO_CLASS
            || interface[6] != AUDIO_CONTROL_SUBCLASS
            || interface[7] != UAC2_PROTOCOL
        {
            continue;
        }

        let body_end = next_interface_offset(configuration_body, offset);
        let body = &configuration_body[offset..body_end];
        // The host stream enters the sink's topology as an input terminal, whose `bCSourceID`
        // names the clock that has to run at 48 kHz.
        let Some(clock_source) = find_cs_interface(body, INPUT_TERMINAL, terminal_link, 8).map(|terminal| terminal[7])
        else {
            continue;
        };
        if clock_source == 0 {
            continue;
        }

        let Some(clock) = find_cs_interface(body, CLOCK_SOURCE, clock_source, 6) else {
            continue;
        };
        // `bmControls` bits 0..1 hold the Clock Frequency Control: 0b01 read-only, 0b11
        // read/write. A clock that cannot even be read leaves the rate unverifiable.
        let frequency_access = clock[5] & 0x03;
        if frequency_access == 0x01 || frequency_access == 0x03 {
            return Some((interface[2], clock_source, frequency_access == 0x03));
        }
    }
    None
}

/// Finds a class-specific interface descriptor of `subtype` whose entity ID at offset 3 matches
/// `entity_id` and which is at least `minimum_length` bytes long.
fn find_cs_interface(descriptors: &[u8], subtype: u8, entity_id: u8, minimum_length: usize) -> Option<&[u8]> {
    let mut offset = 0;
    while let Some((descriptor, next)) = descriptor_at(descriptors, offset) {
        offset = next;
        if descriptor.get(1) == Some(&DESCRIPTOR_CS_INTERFACE)
            && descriptor.get(2) == Some(&subtype)
            && descriptor.len() >= minimum_length
            && descriptor[3] == entity_id
        {
            return Some(descriptor);
        }
    }
    None
}

/// Finds a feature unit fed directly by the streaming input terminal that claims a writable
/// volume control.
///
/// Only the direct `input terminal -> feature unit` hop is followed. That is the topology
/// essentially every playback DAC uses, and guessing through longer chains risks addressing a
/// unit that does not actually attenuate this stream.
fn find_volume_feature_unit(
    configuration_body: &[u8],
    version: AudioClassVersion,
    terminal_link: u8,
) -> Option<FeatureUnitVolume> {
    let protocol = match version {
        AudioClassVersion::Uac1 => UAC1_PROTOCOL,
        AudioClassVersion::Uac2 => UAC2_PROTOCOL,
    };
    let mut offset = 0;
    while let Some((interface, next)) = descriptor_at(configuration_body, offset) {
        offset = next;
        if interface.get(1) != Some(&DESCRIPTOR_INTERFACE)
            || interface.len() < 9
            || interface[5] != AUDIO_CLASS
            || interface[6] != AUDIO_CONTROL_SUBCLASS
            || interface[7] != protocol
        {
            continue;
        }

        let body_end = next_interface_offset(configuration_body, offset);
        let body = &configuration_body[offset..body_end];
        let mut body_offset = 0;
        while let Some((descriptor, body_next)) = descriptor_at(body, body_offset) {
            body_offset = body_next;
            // `bSourceID` sits at offset 4 in both the UAC1 and UAC2 feature unit descriptors.
            if descriptor.get(1) != Some(&DESCRIPTOR_CS_INTERFACE)
                || descriptor.get(2) != Some(&FEATURE_UNIT)
                || descriptor.len() < 6
                || descriptor[4] != terminal_link
            {
                continue;
            }
            if let Some(master) = feature_unit_volume_channel(version, descriptor) {
                return Some(FeatureUnitVolume {
                    control_interface: interface[2],
                    unit_id: descriptor[3],
                    master,
                });
            }
        }
    }
    None
}

/// Reports whether a feature unit has a writable volume control, and whether it is on the master
/// channel rather than the first two addressable channels.
fn feature_unit_volume_channel(version: AudioClassVersion, descriptor: &[u8]) -> Option<bool> {
    let writable: fn(&[u8], usize) -> bool = match version {
        AudioClassVersion::Uac1 => |controls, size| {
            controls[..size]
                .first()
                .is_some_and(|bits| bits & UAC1_VOLUME_CONTROL != 0)
        },
        AudioClassVersion::Uac2 => |controls, _| {
            let bits = u32::from_le_bytes([controls[0], controls[1], controls[2], controls[3]]);
            (bits >> UAC2_VOLUME_CONTROL_SHIFT) & UAC2_CONTROL_READ_WRITE == UAC2_CONTROL_READ_WRITE
        },
    };
    // UAC1 sizes its control bitmap with `bControlSize` and starts it after that field; UAC2
    // always uses four-byte entries beginning immediately after `bSourceID`.
    let (controls_offset, control_size) = match version {
        AudioClassVersion::Uac1 => (6, usize::from(descriptor[5])),
        AudioClassVersion::Uac2 => (5, 4),
    };
    if control_size == 0 {
        return None;
    }
    let controls = descriptor.get(controls_offset..)?;
    // The trailing `iFeature` byte is not part of the bitmap.
    let channel_count = controls.len().saturating_sub(1) / control_size;
    let channel_writable = |channel: usize| {
        channel < channel_count
            && controls
                .get(channel * control_size..(channel + 1) * control_size)
                .is_some_and(|bits| writable(bits, control_size))
    };

    if channel_writable(0) {
        Some(true)
    } else if channel_writable(1) && channel_writable(2) {
        Some(false)
    } else {
        None
    }
}

fn find_endpoint(descriptors: &[u8], input: bool, usage: u8) -> Option<AudioEndpoint> {
    let mut offset = 0;
    while let Some((descriptor, next)) = descriptor_at(descriptors, offset) {
        offset = next;
        if descriptor.get(1) != Some(&DESCRIPTOR_ENDPOINT) || descriptor.len() < 7 {
            continue;
        }
        let address = descriptor[2];
        let attributes = descriptor[3];
        if address & 0x80 != if input { 0x80 } else { 0 }
            || attributes & 0x03 != ENDPOINT_ISOCHRONOUS
            || attributes & ENDPOINT_USAGE_MASK != usage
        {
            continue;
        }
        return Some(AudioEndpoint {
            address,
            attributes,
            max_packet_size: u16::from_le_bytes([descriptor[4], descriptor[5]]) & 0x07ff,
            interval: descriptor[6],
        });
    }
    None
}

/// Reports whether the UAC1 class-specific endpoint descriptor claims a Sampling Frequency
/// Control, which is the only way to move a UAC1 endpoint onto 48 kHz.
fn has_uac1_sample_rate_control(descriptors: &[u8]) -> bool {
    let mut offset = 0;
    while let Some((descriptor, next)) = descriptor_at(descriptors, offset) {
        offset = next;
        if descriptor.get(1) == Some(&DESCRIPTOR_CS_ENDPOINT)
            && descriptor.get(2) == Some(&AS_GENERAL)
            && descriptor.len() >= 4
            && descriptor[3] & 1 != 0
        {
            return true;
        }
    }
    false
}

fn next_interface_offset(buffer: &[u8], start: usize) -> usize {
    let mut offset = start;
    while let Some((descriptor, next)) = descriptor_at(buffer, offset) {
        if descriptor.get(1) == Some(&DESCRIPTOR_INTERFACE) {
            return offset;
        }
        offset = next;
    }
    buffer.len()
}

/// Returns the descriptor beginning at `offset` and the offset of the next one, or `None` once
/// the buffer ends or a `bLength` runs past it.
fn descriptor_at(buffer: &[u8], offset: usize) -> Option<(&[u8], usize)> {
    let length = usize::from(*buffer.get(offset)?);
    if length < 2 {
        return None;
    }
    let end = offset.checked_add(length)?;
    Some((buffer.get(offset..end)?, end))
}

fn read_u24(bytes: &[u8]) -> u32 {
    u32::from(bytes[0]) | (u32::from(bytes[1]) << 8) | (u32::from(bytes[2]) << 16)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use super::*;

    /// UAC1 streaming interface 4 alt 3: PCM, stereo, 16-bit, one discrete 48 kHz rate, adaptive
    /// 192-byte OUT endpoint 0x02, and a class-specific endpoint with no sample-rate control.
    const UAC1_ADAPTIVE_48K: &[u8] = &[
        9, 4, 4, 0, 0, 1, 2, 0, 0, // Streaming alt zero.
        9, 4, 4, 3, 1, 1, 2, 0, 0, // Streaming alt three.
        7, 0x24, 1, 2, 1, 1, 0, // PCM AS general, terminal link 2.
        11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00, // Type I stereo S16, discrete 48 kHz.
        9, 5, 0x02, 0x09, 0xc0, 0x00, 1, 0, 0, // Adaptive OUT, 192 bytes.
        7, 0x25, 1, 0, 0, 0, 0, // Endpoint without a sample-rate control.
    ];

    /// UAC2 control interface 6 with input terminal 9 fed by read/write clock 7, plus streaming
    /// interface 8 alt 2 with an asynchronous OUT endpoint and a 16.16 feedback endpoint.
    const UAC2_ASYNC_48K: &[u8] = &[
        9, 4, 6, 0, 0, 1, 1, 0x20, 0, // Audio control interface 6.
        17, 0x24, 2, 9, 1, 1, 0, 7, 2, 3, 0, 0, 0, 0, 0, 0, 0, // Input terminal 9 -> clock 7.
        8, 0x24, 0x0a, 7, 0, 3, 0, 0, // Clock source 7, frequency read/write.
        9, 4, 8, 2, 2, 1, 2, 0x20, 0, // Streaming interface 8 alt 2.
        16, 0x24, 1, 9, 0, 1, 1, 0, 0, 0, 2, 3, 0, 0, 0, 0, // PCM stereo, terminal link 9.
        6, 0x24, 2, 1, 2, 16, // Type I, two-byte subslot, 16-bit.
        7, 5, 0x03, 0x05, 0xd0, 0x00, 1, // Asynchronous OUT, 208 bytes.
        7, 5, 0x84, 0x11, 4, 0, 4, // Explicit feedback IN, four bytes, every 8 frames.
    ];

    /// Copies `fixture` with one byte replaced, asserting the byte currently holds `expected` so
    /// that a mis-computed descriptor offset fails the test instead of silently testing nothing.
    fn patched(fixture: &[u8], index: usize, expected: u8, replacement: u8) -> Vec<u8> {
        assert_eq!(
            fixture[index], expected,
            "fixture offset {index} is not the intended field"
        );
        let mut patched = fixture.to_vec();
        patched[index] = replacement;
        patched
    }

    /// Builds a one-alternate UAC1 streaming interface around `format` so each test varies only
    /// the field it is about.
    fn uac1_alternate(format: &[u8], endpoint_attributes: u8, max_packet_size: u16) -> Vec<u8> {
        let [size_low, size_high] = max_packet_size.to_le_bytes();
        let mut descriptors = Vec::new();
        descriptors.extend_from_slice(&[9, 4, 1, 1, 1, 1, 2, 0, 0]);
        descriptors.extend_from_slice(&[7, 0x24, 1, 2, 1, 1, 0]);
        descriptors.extend_from_slice(format);
        descriptors.extend_from_slice(&[9, 5, 0x02, endpoint_attributes, size_low, size_high, 1, 0, 0]);
        descriptors
    }

    /// UAC1 control interface 0 with input terminal 1 feeding feature unit 2, whose master
    /// channel claims mute and volume, plus a matching streaming interface 1 alt 1.
    const UAC1_WITH_MASTER_VOLUME: &[u8] = &[
        9, 4, 0, 0, 0, 1, 1, 0, 0, // Audio control interface 0.
        12, 0x24, 2, 1, 1, 1, 0, 2, 3, 0, 0, 0, // Input terminal 1, USB streaming, stereo.
        10, 0x24, 6, 2, 1, 1, 0x03, 0, 0, 0, // Feature unit 2 <- terminal 1, master mute+volume.
        9, 4, 1, 0, 0, 1, 2, 0, 0, // Streaming alt zero.
        9, 4, 1, 1, 1, 1, 2, 0, 0, // Streaming alt one.
        7, 0x24, 1, 1, 1, 1, 0, // PCM AS general, terminal link 1.
        11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00, // Type I stereo S16, 48 kHz.
        9, 5, 0x02, 0x09, 0xc0, 0x00, 1, 0, 0, // Adaptive OUT, 192 bytes.
    ];

    /// Splices a feature unit into the audio control interface of [`UAC2_ASYNC_48K`], which ends
    /// where the streaming interface descriptor begins.
    fn uac2_with_feature_unit(feature_unit: &[u8]) -> Vec<u8> {
        const STREAMING_INTERFACE: usize = 34;
        assert_eq!(
            &UAC2_ASYNC_48K[STREAMING_INTERFACE..STREAMING_INTERFACE + 2],
            &[9, 4],
            "offset is not the start of the streaming interface descriptor"
        );
        let mut descriptors = UAC2_ASYNC_48K[..STREAMING_INTERFACE].to_vec();
        descriptors.extend_from_slice(feature_unit);
        descriptors.extend_from_slice(&UAC2_ASYNC_48K[STREAMING_INTERFACE..]);
        descriptors
    }

    #[test]
    fn selects_uac1_fixed_rate_on_arbitrary_interface() {
        assert_eq!(
            find_playback(UAC1_ADAPTIVE_48K),
            Some(PlaybackConfig {
                version: AudioClassVersion::Uac1,
                interface_number: 4,
                alternate_setting: 3,
                output: AudioEndpoint {
                    address: 0x02,
                    attributes: 0x09,
                    max_packet_size: 192,
                    interval: 1,
                },
                feedback: None,
                uac1_sample_rate_control: false,
                uac2_control_interface: 0,
                uac2_clock_source: 0,
                uac2_clock_writable: false,
                volume: None,
            })
        );
    }

    #[test]
    fn selects_uac2_clock_and_explicit_feedback() {
        assert_eq!(
            find_playback(UAC2_ASYNC_48K),
            Some(PlaybackConfig {
                version: AudioClassVersion::Uac2,
                interface_number: 8,
                alternate_setting: 2,
                output: AudioEndpoint {
                    address: 0x03,
                    attributes: 0x05,
                    max_packet_size: 208,
                    interval: 1,
                },
                feedback: Some(AudioEndpoint {
                    address: 0x84,
                    attributes: 0x11,
                    max_packet_size: 4,
                    interval: 4,
                }),
                uac1_sample_rate_control: false,
                uac2_control_interface: 6,
                uac2_clock_source: 7,
                uac2_clock_writable: true,
                volume: None,
            })
        );
    }

    #[test]
    fn reports_uac1_endpoint_sample_rate_control() {
        // Set bit 0 of the class-specific endpoint's bmAttributes.
        let descriptors = patched(UAC1_ADAPTIVE_48K, 48, 0, 1);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert!(selected.uac1_sample_rate_control);
    }

    #[test]
    fn accepts_uac1_continuous_rate_range_covering_48_khz() {
        let format = [14, 0x24, 2, 1, 2, 2, 16, 0, 0x40, 0x1f, 0x00, 0x00, 0x77, 0x01];

        let selected = find_playback(&uac1_alternate(&format, 0x09, 192)).expect("continuous 8k-96k range");
        assert_eq!(selected.version, AudioClassVersion::Uac1);
    }

    #[test]
    fn rejects_uac1_continuous_rate_range_below_48_khz() {
        let format = [14, 0x24, 2, 1, 2, 2, 16, 0, 0x40, 0x1f, 0x00, 0x44, 0xac, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x09, 192)), None);
    }

    #[test]
    fn accepts_uac1_discrete_rate_list_containing_48_khz() {
        let format = [
            17, 0x24, 2, 1, 2, 2, 16, 3, // Three discrete rates.
            0x44, 0xac, 0x00, // 44.1 kHz.
            0x80, 0xbb, 0x00, // 48 kHz.
            0x00, 0x77, 0x01, // 96 kHz.
        ];

        assert!(find_playback(&uac1_alternate(&format, 0x09, 192)).is_some());
    }

    #[test]
    fn rejects_uac1_alternate_without_48_khz() {
        let format = [11, 0x24, 2, 1, 2, 2, 16, 1, 0x44, 0xac, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x09, 192)), None);
    }

    #[test]
    fn rejects_uac1_alternate_with_24_bit_samples() {
        let format = [11, 0x24, 2, 1, 2, 3, 24, 1, 0x80, 0xbb, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x09, 192)), None);
    }

    #[test]
    fn rejects_uac1_mono_alternate() {
        let format = [11, 0x24, 2, 1, 1, 2, 16, 1, 0x80, 0xbb, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x09, 192)), None);
    }

    #[test]
    fn rejects_output_packet_too_small_for_one_millisecond() {
        let format = [11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x09, 128)), None);
    }

    #[test]
    fn rejects_asynchronous_output_without_feedback() {
        let format = [11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00];

        assert_eq!(find_playback(&uac1_alternate(&format, 0x05, 208)), None);
    }

    #[test]
    fn accepts_synchronous_output_without_feedback() {
        let format = [11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00];

        let selected = find_playback(&uac1_alternate(&format, 0x0d, 192)).expect("synchronous alternate");
        assert!(!selected.output.is_asynchronous());
        assert_eq!(selected.feedback, None);
    }

    #[test]
    fn ignores_feedback_endpoint_with_implausible_packet_size() {
        let mut descriptors = uac1_alternate(&[11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00], 0x09, 192);
        // Feedback IN endpoint claiming 16 bytes, which no explicit-feedback format uses.
        descriptors.extend_from_slice(&[9, 5, 0x81, 0x11, 16, 0, 1, 0, 0]);

        let selected = find_playback(&descriptors).expect("adaptive alternate");
        assert_eq!(selected.feedback, None);
    }

    #[test]
    fn accepts_three_byte_full_speed_feedback_endpoint() {
        let mut descriptors = uac1_alternate(&[11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00], 0x05, 192);
        descriptors.extend_from_slice(&[9, 5, 0x81, 0x11, 3, 0, 1, 0, 0]);

        let selected = find_playback(&descriptors).expect("asynchronous alternate with 10.14 feedback");
        assert_eq!(selected.feedback.map(|endpoint| endpoint.max_packet_size), Some(3));
    }

    #[test]
    fn skips_alternate_setting_zero() {
        // Alt zero carrying an otherwise valid format must never be selected.
        let descriptors = [
            9, 4, 1, 0, 1, 1, 2, 0, 0, //
            7, 0x24, 1, 2, 1, 1, 0, //
            11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00, //
            9, 5, 0x02, 0x09, 0xc0, 0x00, 1, 0, 0,
        ];

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn skips_incompatible_alternate_and_selects_a_later_one() {
        // Alt one is 24-bit and must be passed over.
        let mut descriptors = uac1_alternate(&[11, 0x24, 2, 1, 2, 3, 24, 1, 0x80, 0xbb, 0x00], 0x09, 288);
        descriptors.extend_from_slice(UAC1_ADAPTIVE_48K);

        let selected = find_playback(&descriptors).expect("second alternate");
        assert_eq!((selected.interface_number, selected.alternate_setting), (4, 3));
    }

    #[test]
    fn rejects_capture_only_interface() {
        let descriptors = [
            9, 4, 1, 1, 1, 1, 2, 0, 0, //
            7, 0x24, 1, 2, 1, 1, 0, //
            11, 0x24, 2, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00, //
            9, 5, 0x82, 0x09, 0xc0, 0x00, 1, 0, 0, // Isochronous IN, not a playback path.
        ];

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn rejects_vendor_specific_interface() {
        // bInterfaceClass of the only alternate carrying audio endpoints.
        let descriptors = patched(UAC1_ADAPTIVE_48K, 14, 1, 0xff);

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn accepts_read_only_uac2_clock() {
        // bmControls of clock source 7: read-only frequency.
        let descriptors = patched(UAC2_ASYNC_48K, 31, 3, 1);

        let selected = find_playback(&descriptors).expect("read-only clock");
        assert_eq!(selected.uac2_clock_source, 7);
        assert!(!selected.uac2_clock_writable);
    }

    #[test]
    fn rejects_uac2_clock_without_frequency_control() {
        let descriptors = patched(UAC2_ASYNC_48K, 31, 3, 0);

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn rejects_uac2_stream_whose_terminal_is_missing() {
        // Point the streaming interface at a terminal the control interface does not define.
        let descriptors = patched(UAC2_ASYNC_48K, 46, 9, 5);

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn rejects_uac2_terminal_naming_an_absent_clock() {
        // Input terminal 9 now sources a clock entity that is not declared.
        let descriptors = patched(UAC2_ASYNC_48K, 16, 7, 4);

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn rejects_uac2_alternate_with_24_bit_samples() {
        let mut descriptors = patched(UAC2_ASYNC_48K, 63, 2, 3); // bSubslotSize
        descriptors = patched(&descriptors, 64, 16, 24); // bBitResolution

        assert_eq!(find_playback(&descriptors), None);
    }

    #[test]
    fn truncated_descriptors_never_panic() {
        for fixture in [UAC1_ADAPTIVE_48K, UAC2_ASYNC_48K] {
            for end in 0..=fixture.len() {
                let _ = find_playback(&fixture[..end]);
            }
        }
    }

    #[test]
    fn corrupted_descriptor_lengths_never_panic() {
        for fixture in [UAC1_ADAPTIVE_48K, UAC2_ASYNC_48K] {
            for index in 0..fixture.len() {
                for length in [0_u8, 1, 2, 0x7f, 0xff] {
                    let mut damaged = fixture.to_vec();
                    damaged[index] = length;
                    let _ = find_playback(&damaged);
                }
            }
        }
    }

    #[test]
    fn finds_uac1_master_volume_feature_unit() {
        let selected = find_playback(UAC1_WITH_MASTER_VOLUME).expect("48 kHz stereo alternate");

        assert_eq!(
            selected.volume,
            Some(FeatureUnitVolume {
                control_interface: 0,
                unit_id: 2,
                master: true,
            })
        );
    }

    #[test]
    fn ignores_uac1_feature_unit_offering_only_mute() {
        // bmaControls for the master channel with the volume bit cleared.
        let descriptors = patched(UAC1_WITH_MASTER_VOLUME, 27, 0x03, 0x01);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(selected.volume, None);
    }

    #[test]
    fn ignores_uac1_feature_unit_sourced_from_another_terminal() {
        // bSourceID of feature unit 2 now names a terminal that does not carry this stream.
        let descriptors = patched(UAC1_WITH_MASTER_VOLUME, 25, 1, 9);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(selected.volume, None);
    }

    #[test]
    fn finds_uac1_per_channel_volume_feature_unit() {
        // Clear the master channel's controls and grant volume on channels one and two.
        let mut descriptors = patched(UAC1_WITH_MASTER_VOLUME, 27, 0x03, 0x00);
        descriptors = patched(&descriptors, 28, 0x00, 0x02);
        descriptors = patched(&descriptors, 29, 0x00, 0x02);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(
            selected.volume,
            Some(FeatureUnitVolume {
                control_interface: 0,
                unit_id: 2,
                master: false,
            })
        );
    }

    #[test]
    fn finds_uac2_master_volume_feature_unit() {
        // Feature unit 3 <- terminal 9, master volume read/write (bits 2..3 set).
        let descriptors = uac2_with_feature_unit(&[18, 0x24, 6, 3, 9, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(
            selected.volume,
            Some(FeatureUnitVolume {
                control_interface: 6,
                unit_id: 3,
                master: true,
            })
        );
    }

    #[test]
    fn finds_uac2_per_channel_volume_feature_unit() {
        let descriptors = uac2_with_feature_unit(&[18, 0x24, 6, 3, 9, 0, 0, 0, 0, 0x0c, 0, 0, 0, 0x0c, 0, 0, 0, 0]);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(selected.volume.map(|volume| volume.master), Some(false));
    }

    #[test]
    fn ignores_uac2_read_only_volume_control() {
        // Volume access bits 2..3 set to 0b01, which is read-only.
        let descriptors = uac2_with_feature_unit(&[18, 0x24, 6, 3, 9, 0x04, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(selected.volume, None);
    }

    #[test]
    fn ignores_uac2_feature_unit_with_only_one_channel_writable() {
        let descriptors = uac2_with_feature_unit(&[18, 0x24, 6, 3, 9, 0, 0, 0, 0, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0]);

        let selected = find_playback(&descriptors).expect("48 kHz stereo alternate");
        assert_eq!(selected.volume, None);
    }

    #[test]
    fn feature_unit_discovery_never_panics_on_damage() {
        let fixture = uac2_with_feature_unit(&[18, 0x24, 6, 3, 9, 0x0c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        for source in [UAC1_WITH_MASTER_VOLUME, fixture.as_slice()] {
            for end in 0..=source.len() {
                let _ = find_playback(&source[..end]);
            }
            for index in 0..source.len() {
                for length in [0_u8, 1, 2, 0x7f, 0xff] {
                    let mut damaged = source.to_vec();
                    damaged[index] = length;
                    let _ = find_playback(&damaged);
                }
            }
        }
    }

    /// Decibels for a Q15 gain, used to assert the host curve rather than restating the table.
    fn gain_db(gain_q15: u16) -> f64 {
        20.0 * f64::from(gain_q15).log10() - 20.0 * f64::from(UNITY_GAIN_Q15).log10()
    }

    #[test]
    fn host_gain_keeps_the_lower_half_audible() {
        assert_eq!(host_gain_q15(MAXIMUM_LEVEL), UNITY_GAIN_Q15);
        // The midpoint must remain comfortably audible on quiet source material.
        assert!((gain_db(host_gain_q15(128)) + 15.0).abs() < 1.0);
        assert!((gain_db(host_gain_q15(204)) + 6.0).abs() < 1.0);
        assert!((gain_db(host_gain_q15(1)) + 30.0).abs() < 1.0);
    }

    #[test]
    fn host_gain_is_monotonic_across_every_level() {
        let mut previous = 0;
        for level in 0..=MAXIMUM_LEVEL {
            let gain = host_gain_q15(level);
            assert!(gain >= previous, "level {level} lowered the gain");
            assert!(gain <= UNITY_GAIN_Q15, "level {level} exceeded unity");
            previous = gain;
        }
    }

    #[test]
    fn host_gain_tracks_the_decibel_curve_between_table_entries() {
        // Interpolation error has to stay well under what a listener notices.
        for level in 1..=MAXIMUM_LEVEL {
            let expected = -f64::from(VOLUME_RANGE_DB) * (1.0 - f64::from(level) / 255.0);
            let error = (gain_db(host_gain_q15(level)) - expected).abs();
            assert!(error < 0.4, "level {level} was off by {error} dB");
        }
    }

    #[test]
    fn device_volume_is_linear_in_decibels_over_the_usable_range() {
        // A device reporting a -127.5 dB floor must not have its whole floor mapped onto the
        // scale, or most of the control is inaudible.
        let minimum = -127 * 256;
        let maximum = 0;

        assert_eq!(device_volume_setting(MAXIMUM_LEVEL, minimum, maximum), 0);
        assert_eq!(device_volume_setting(128, minimum, maximum), -15 * 256 + 15);
        assert_eq!(device_volume_setting(204, minimum, maximum), -6 * 256);
    }

    #[test]
    fn device_volume_never_leaves_the_reported_range() {
        for (minimum, maximum) in [(-127 * 256, 0), (-40 * 256, 0), (-20 * 256, 10 * 256), (-256, 0)] {
            for level in 1..=MAXIMUM_LEVEL {
                let setting = device_volume_setting(level, minimum, maximum);
                assert!(
                    (minimum..=maximum).contains(&setting),
                    "level {level} produced {setting} outside {minimum}..={maximum}"
                );
            }
        }
    }

    #[test]
    fn device_volume_respects_a_narrow_device_range() {
        // A device that only attenuates by 20 dB must still reach both of its own limits.
        let (minimum, maximum) = (-20 * 256, 0);

        assert_eq!(device_volume_setting(MAXIMUM_LEVEL, minimum, maximum), maximum);
        assert_eq!(device_volume_setting(1, minimum, maximum), minimum + 20 * 256 / 255);
    }

    #[test]
    fn level_zero_is_silence_on_both_paths() {
        assert_eq!(device_volume_setting(0, -127 * 256, 0), VOLUME_SILENCE);
        assert_eq!(host_gain_q15(0), 0);
    }

    #[test]
    fn empty_configuration_selects_nothing() {
        assert_eq!(find_playback(&[]), None);
    }
}
