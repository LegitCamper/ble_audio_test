#![no_std]
#![deny(missing_docs)]

//! Bounded serial framing for forwarding audio between two MCUs.
//!
//! A frame is COBS encoded and terminated by a zero byte, so a receiver can recover after a
//! truncated or corrupt UART transfer. The decoded body ends in a CRC-16/CCITT-FALSE checksum.
//! All storage is fixed-size and owned by the caller or the parser; no allocator is required.

use core::fmt;

/// Protocol version emitted by this crate.
pub const VERSION: u8 = 1;
/// Largest LC3 payload accepted by the transport.
pub const MAX_LC3_FRAME_BYTES: usize = 155;
/// Bytes in one packed 48 kHz, 10 ms, signed 12-bit mono PCM frame.
pub const PCM_S12_MONO_FRAME_BYTES: usize = 720;
/// Samples in one 48 kHz, 10 ms mono PCM frame.
pub const PCM_MONO_SAMPLES_PER_FRAME: usize = 480;
/// Largest audio payload accepted by the transport.
pub const MAX_AUDIO_PAYLOAD_BYTES: usize = PCM_S12_MONO_FRAME_BYTES;

const HEADER_BYTES: usize = 18;
const CRC_BYTES: usize = 2;
const MAX_RAW_FRAME_BYTES: usize = HEADER_BYTES + MAX_AUDIO_PAYLOAD_BYTES + CRC_BYTES;
const MAX_COBS_FRAME_BYTES: usize = MAX_RAW_FRAME_BYTES + MAX_RAW_FRAME_BYTES / 254 + 1;
/// Largest encoded frame, including its trailing zero delimiter.
pub const MAX_WIRE_FRAME_BYTES: usize = MAX_COBS_FRAME_BYTES + 1;

/// Encoding used by an audio-frame payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AudioEncoding {
    /// One encoded LC3 channel.
    Lc3 = 1,
    /// Two signed 12-bit little-endian PCM samples packed into each three bytes.
    PcmS12LeMono = 2,
}

impl TryFrom<u8> for AudioEncoding {
    type Error = DecodeError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Lc3),
            2 => Ok(Self::PcmS12LeMono),
            _ => Err(DecodeError::UnsupportedKind(value)),
        }
    }
}

/// Logical channel carried by one LC3 frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Channel {
    /// Front-left audio.
    Left = 0,
    /// Front-right audio.
    Right = 1,
    /// A channel explicitly negotiated as mono.
    Mono = 2,
    /// No usable channel allocation was supplied by the peer.
    Unknown = 255,
}

impl TryFrom<u8> for Channel {
    type Error = DecodeError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Left),
            1 => Ok(Self::Right),
            2 => Ok(Self::Mono),
            255 => Ok(Self::Unknown),
            _ => Err(DecodeError::InvalidChannel(value)),
        }
    }
}

/// Negotiated duration of one LC3 frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FrameDuration {
    /// 7.5 milliseconds.
    Millis7_5 = 0,
    /// 10 milliseconds.
    Millis10 = 1,
}

impl TryFrom<u8> for FrameDuration {
    type Error = DecodeError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Millis7_5),
            1 => Ok(Self::Millis10),
            _ => Err(DecodeError::InvalidFrameDuration(value)),
        }
    }
}

/// Flags describing exceptional conditions associated with a frame.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FrameFlags(u8);

impl FrameFlags {
    /// The sender observed a gap or queue overflow before this frame.
    pub const DISCONTINUITY: Self = Self(1 << 0);
    /// This is the first frame emitted for the given ASE after streaming began.
    pub const STREAM_START: Self = Self(1 << 1);

    /// Creates an empty flag set.
    #[must_use]
    pub const fn empty() -> Self {
        Self(0)
    }

    /// Creates a flag set from its wire representation.
    #[must_use]
    pub const fn from_bits_truncate(bits: u8) -> Self {
        Self(bits & (Self::DISCONTINUITY.0 | Self::STREAM_START.0))
    }

    /// Returns the wire representation.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns whether all flags in `other` are present.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Returns the union of two flag sets.
    #[must_use]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Metadata prepended to one audio frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FrameMeta {
    /// Encoding of the frame payload.
    pub encoding: AudioEncoding,
    /// Channel assignment derived from the ASE audio allocation.
    pub channel: Channel,
    /// ASCS endpoint identifier assigned by the LE Audio peer.
    pub ase_id: u8,
    /// Per-ASE wrapping sequence number.
    pub sequence: u16,
    /// Sender-local receive timestamp in microseconds, wrapping at `u32::MAX`.
    pub timestamp_us: u32,
    /// Negotiated sample rate.
    pub sample_rate_hz: u32,
    /// Negotiated LC3 frame duration.
    pub frame_duration: FrameDuration,
    /// Loss and stream-boundary indicators.
    pub flags: FrameFlags,
}

/// One decoded, owned audio frame.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecodedFrame {
    /// Frame metadata.
    pub meta: FrameMeta,
    payload: [u8; MAX_AUDIO_PAYLOAD_BYTES],
    payload_len: usize,
}

impl DecodedFrame {
    /// Returns the audio payload.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
}

/// One COBS-encoded frame ready to write to a UART, including the zero delimiter.
pub struct EncodedFrame {
    bytes: [u8; MAX_WIRE_FRAME_BYTES],
    len: usize,
}

impl EncodedFrame {
    /// Returns the exact bytes to transmit.
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Failure while constructing a wire frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EncodeError {
    /// The audio frame exceeds the limit for its encoding.
    PayloadTooLong(usize),
}

impl fmt::Display for EncodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLong(len) => write!(formatter, "audio payload is too long: {len} bytes"),
        }
    }
}

/// Failure while packing or unpacking a fixed-duration PCM frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PcmPackingError {
    /// A PCM frame did not contain exactly [`PCM_MONO_SAMPLES_PER_FRAME`] samples.
    InvalidSampleCount(usize),
    /// A packed PCM frame did not contain exactly [`PCM_S12_MONO_FRAME_BYTES`] bytes.
    InvalidByteCount(usize),
}

impl fmt::Display for PcmPackingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSampleCount(count) => write!(formatter, "invalid PCM sample count: {count}"),
            Self::InvalidByteCount(count) => write!(formatter, "invalid packed PCM byte count: {count}"),
        }
    }
}

/// Quantizes signed 16-bit PCM to signed 12-bit and packs two samples into three bytes.
///
/// # Errors
///
/// Returns [`PcmPackingError::InvalidSampleCount`] unless `samples` is one 48 kHz, 10 ms
/// mono frame.
pub fn pack_pcm_s12_le(samples: &[i16]) -> Result<[u8; PCM_S12_MONO_FRAME_BYTES], PcmPackingError> {
    if samples.len() != PCM_MONO_SAMPLES_PER_FRAME {
        return Err(PcmPackingError::InvalidSampleCount(samples.len()));
    }

    let mut packed = [0_u8; PCM_S12_MONO_FRAME_BYTES];
    for (input, output) in samples.chunks_exact(2).zip(packed.chunks_exact_mut(3)) {
        let first = (input[0] >> 4).to_le_bytes();
        let second = (input[1] >> 4).to_le_bytes();
        output[0] = first[0];
        output[1] = (first[1] & 0x0f) | (second[0] << 4);
        output[2] = (second[0] >> 4) | (second[1] << 4);
    }
    Ok(packed)
}

/// Unpacks signed 12-bit PCM and expands it to signed 16-bit samples.
///
/// The low four output bits are zero because they were removed during packing.
///
/// # Errors
///
/// Returns [`PcmPackingError::InvalidByteCount`] unless `packed` is one complete frame.
pub fn unpack_pcm_s12_le(packed: &[u8]) -> Result<[i16; PCM_MONO_SAMPLES_PER_FRAME], PcmPackingError> {
    if packed.len() != PCM_S12_MONO_FRAME_BYTES {
        return Err(PcmPackingError::InvalidByteCount(packed.len()));
    }

    let mut samples = [0_i16; PCM_MONO_SAMPLES_PER_FRAME];
    for (input, output) in packed.chunks_exact(3).zip(samples.chunks_exact_mut(2)) {
        let first_high = input[1] & 0x0f;
        let second_high = input[2] >> 4;
        let first_sign = if first_high & 0x08 == 0 { 0 } else { 0xf0 };
        let second_sign = if second_high & 0x08 == 0 { 0 } else { 0xf0 };
        let first = i16::from_le_bytes([input[0], first_high | first_sign]);
        let second_low = (input[1] >> 4) | (input[2] << 4);
        let second = i16::from_le_bytes([second_low, second_high | second_sign]);
        output[0] = first << 4;
        output[1] = second << 4;
    }
    Ok(samples)
}

/// Failure while decoding a wire frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Bytes before a delimiter exceeded the bounded parser capacity.
    FrameTooLong,
    /// The COBS structure was invalid.
    MalformedCobs,
    /// The decoded body cannot contain the fixed header and CRC.
    FrameTooShort(usize),
    /// The peer uses an unsupported protocol version.
    UnsupportedVersion(u8),
    /// The frame kind is not a supported audio encoding.
    UnsupportedKind(u8),
    /// The logical channel byte was invalid.
    InvalidChannel(u8),
    /// The frame-duration byte was invalid.
    InvalidFrameDuration(u8),
    /// The header payload length disagrees with the decoded body length.
    LengthMismatch,
    /// The body checksum was incorrect.
    CrcMismatch,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FrameTooLong => formatter.write_str("wire frame is too long"),
            Self::MalformedCobs => formatter.write_str("malformed COBS frame"),
            Self::FrameTooShort(len) => write!(formatter, "decoded frame is too short: {len} bytes"),
            Self::UnsupportedVersion(version) => write!(formatter, "unsupported protocol version: {version}"),
            Self::UnsupportedKind(kind) => write!(formatter, "unsupported frame kind: {kind}"),
            Self::InvalidChannel(channel) => write!(formatter, "invalid channel: {channel}"),
            Self::InvalidFrameDuration(duration) => write!(formatter, "invalid frame duration: {duration}"),
            Self::LengthMismatch => formatter.write_str("payload length does not match frame length"),
            Self::CrcMismatch => formatter.write_str("CRC mismatch"),
        }
    }
}

/// Encodes one audio frame into a self-contained UART packet.
///
/// # Errors
///
/// Returns [`EncodeError::PayloadTooLong`] when `payload` is larger than the protocol limit.
pub fn encode(meta: FrameMeta, payload: &[u8]) -> Result<EncodedFrame, EncodeError> {
    let payload_limit = match meta.encoding {
        AudioEncoding::Lc3 => MAX_LC3_FRAME_BYTES,
        AudioEncoding::PcmS12LeMono => PCM_S12_MONO_FRAME_BYTES,
    };
    if payload.len() > payload_limit {
        return Err(EncodeError::PayloadTooLong(payload.len()));
    }

    let mut raw = [0_u8; MAX_RAW_FRAME_BYTES];
    raw[0] = VERSION;
    raw[1] = meta.encoding as u8;
    raw[2] = meta.flags.bits();
    raw[3] = meta.channel as u8;
    raw[4] = meta.ase_id;
    raw[5] = meta.frame_duration as u8;
    raw[6..8].copy_from_slice(&meta.sequence.to_le_bytes());
    raw[8..12].copy_from_slice(&meta.timestamp_us.to_le_bytes());
    raw[12..16].copy_from_slice(&meta.sample_rate_hz.to_le_bytes());
    let payload_len = u16::try_from(payload.len()).map_err(|_| EncodeError::PayloadTooLong(payload.len()))?;
    raw[16..18].copy_from_slice(&payload_len.to_le_bytes());
    raw[HEADER_BYTES..HEADER_BYTES + payload.len()].copy_from_slice(payload);

    let body_len = HEADER_BYTES + payload.len();
    let crc = crc16_ccitt_false(&raw[..body_len]);
    raw[body_len..body_len + CRC_BYTES].copy_from_slice(&crc.to_le_bytes());

    let mut bytes = [0_u8; MAX_WIRE_FRAME_BYTES];
    let encoded_len = cobs_encode(&raw[..body_len + CRC_BYTES], &mut bytes);
    bytes[encoded_len] = 0;
    Ok(EncodedFrame {
        bytes,
        len: encoded_len + 1,
    })
}

/// Incremental decoder for a UART byte stream.
pub struct StreamDecoder {
    encoded: [u8; MAX_COBS_FRAME_BYTES],
    len: usize,
    discarding: bool,
}

impl Default for StreamDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamDecoder {
    /// Creates an empty stream decoder.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            encoded: [0; MAX_COBS_FRAME_BYTES],
            len: 0,
            discarding: false,
        }
    }

    /// Consumes one UART byte and returns a frame when a delimiter completes it.
    ///
    /// Empty delimiters are ignored. After an oversized frame, input is discarded through the
    /// next delimiter so the following valid frame starts from a known boundary.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError`] when a delimited frame is oversized, malformed, corrupt, or uses
    /// unsupported field values.
    pub fn push(&mut self, byte: u8) -> Result<Option<DecodedFrame>, DecodeError> {
        if byte != 0 {
            if self.discarding {
                return Ok(None);
            }
            if self.len == self.encoded.len() {
                self.len = 0;
                self.discarding = true;
                return Ok(None);
            }
            self.encoded[self.len] = byte;
            self.len += 1;
            return Ok(None);
        }

        if self.discarding {
            self.discarding = false;
            self.len = 0;
            return Err(DecodeError::FrameTooLong);
        }
        if self.len == 0 {
            return Ok(None);
        }

        let encoded_len = self.len;
        self.len = 0;
        let mut raw = [0_u8; MAX_RAW_FRAME_BYTES];
        let raw_len = cobs_decode(&self.encoded[..encoded_len], &mut raw)?;
        decode_raw(&raw[..raw_len]).map(Some)
    }
}

fn decode_raw(raw: &[u8]) -> Result<DecodedFrame, DecodeError> {
    if raw.len() < HEADER_BYTES + CRC_BYTES {
        return Err(DecodeError::FrameTooShort(raw.len()));
    }
    if raw[0] != VERSION {
        return Err(DecodeError::UnsupportedVersion(raw[0]));
    }
    let encoding = AudioEncoding::try_from(raw[1])?;

    let payload_len = u16::from_le_bytes([raw[16], raw[17]]) as usize;
    let payload_limit = match encoding {
        AudioEncoding::Lc3 => MAX_LC3_FRAME_BYTES,
        AudioEncoding::PcmS12LeMono => PCM_S12_MONO_FRAME_BYTES,
    };
    if payload_len > payload_limit || raw.len() != HEADER_BYTES + payload_len + CRC_BYTES {
        return Err(DecodeError::LengthMismatch);
    }

    let crc_offset = HEADER_BYTES + payload_len;
    let expected_crc = u16::from_le_bytes([raw[crc_offset], raw[crc_offset + 1]]);
    if crc16_ccitt_false(&raw[..crc_offset]) != expected_crc {
        return Err(DecodeError::CrcMismatch);
    }

    let mut payload = [0_u8; MAX_AUDIO_PAYLOAD_BYTES];
    payload[..payload_len].copy_from_slice(&raw[HEADER_BYTES..crc_offset]);
    Ok(DecodedFrame {
        meta: FrameMeta {
            encoding,
            channel: Channel::try_from(raw[3])?,
            ase_id: raw[4],
            sequence: u16::from_le_bytes([raw[6], raw[7]]),
            timestamp_us: u32::from_le_bytes([raw[8], raw[9], raw[10], raw[11]]),
            sample_rate_hz: u32::from_le_bytes([raw[12], raw[13], raw[14], raw[15]]),
            frame_duration: FrameDuration::try_from(raw[5])?,
            flags: FrameFlags::from_bits_truncate(raw[2]),
        },
        payload,
        payload_len,
    })
}

fn cobs_encode(input: &[u8], output: &mut [u8]) -> usize {
    let mut read_index = 0;
    let mut write_index = 1;
    let mut code_index = 0;
    let mut code = 1_u8;

    while read_index < input.len() {
        if input[read_index] == 0 {
            output[code_index] = code;
            code_index = write_index;
            write_index += 1;
            code = 1;
        } else {
            output[write_index] = input[read_index];
            write_index += 1;
            code += 1;
            if code == u8::MAX {
                output[code_index] = code;
                code_index = write_index;
                write_index += 1;
                code = 1;
            }
        }
        read_index += 1;
    }
    output[code_index] = code;
    write_index
}

fn cobs_decode(input: &[u8], output: &mut [u8]) -> Result<usize, DecodeError> {
    let mut read_index = 0;
    let mut write_index = 0;

    while read_index < input.len() {
        let code = input[read_index] as usize;
        if code == 0 || read_index + code > input.len() + 1 {
            return Err(DecodeError::MalformedCobs);
        }
        read_index += 1;

        let copy_len = code - 1;
        if write_index + copy_len > output.len() {
            return Err(DecodeError::FrameTooLong);
        }
        output[write_index..write_index + copy_len].copy_from_slice(&input[read_index..read_index + copy_len]);
        write_index += copy_len;
        read_index += copy_len;

        if code != usize::from(u8::MAX) && read_index < input.len() {
            if write_index == output.len() {
                return Err(DecodeError::FrameTooLong);
            }
            output[write_index] = 0;
            write_index += 1;
        }
    }
    Ok(write_index)
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for &byte in bytes {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 == 0 {
                crc << 1
            } else {
                (crc << 1) ^ 0x1021
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    fn example_meta() -> FrameMeta {
        FrameMeta {
            encoding: AudioEncoding::Lc3,
            channel: Channel::Right,
            ase_id: 7,
            sequence: 0xfffe,
            timestamp_us: 123_456,
            sample_rate_hz: 48_000,
            frame_duration: FrameDuration::Millis10,
            flags: FrameFlags::STREAM_START.union(FrameFlags::DISCONTINUITY),
        }
    }

    #[test]
    fn crc_should_match_ccitt_false_check_value() {
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29b1);
    }

    #[test]
    fn stream_decoder_should_round_trip_payload_containing_zeroes() {
        let payload = [0, 1, 0, 2, 3, 0, 4];
        let encoded = encode(example_meta(), &payload).expect("test frame must encode");
        let mut parser = StreamDecoder::new();
        let mut completed_frame = None;
        for &byte in encoded.as_bytes() {
            completed_frame = parser.push(byte).expect("test frame must decode").or(completed_frame);
        }

        let completed_frame = completed_frame.expect("delimiter must complete a frame");
        assert_eq!(
            (completed_frame.meta, completed_frame.payload()),
            (example_meta(), payload.as_slice())
        );
    }

    #[test]
    fn stream_decoder_should_reject_a_corrupt_body() {
        let mut encoded = encode(example_meta(), &[1, 2, 3, 4]).expect("test frame must encode");
        encoded.bytes[5] ^= 0x40;
        let mut parser = StreamDecoder::new();
        let mut result = Ok(None);
        for &byte in encoded.as_bytes() {
            result = parser.push(byte);
        }

        assert_eq!(result, Err(DecodeError::CrcMismatch));
    }

    #[test]
    fn stream_decoder_should_recover_after_an_oversized_frame() {
        let mut parser = StreamDecoder::new();
        for _ in 0..=MAX_COBS_FRAME_BYTES {
            assert_eq!(parser.push(1), Ok(None));
        }
        assert_eq!(parser.push(0), Err(DecodeError::FrameTooLong));

        let encoded = encode(example_meta(), &[9, 8, 7]).expect("test frame must encode");
        let mut completed_frame = None;
        for &byte in encoded.as_bytes() {
            completed_frame = parser.push(byte).expect("next frame must decode").or(completed_frame);
        }
        assert_eq!(completed_frame.expect("frame expected").payload(), &[9, 8, 7]);
    }

    #[test]
    fn encode_should_reject_a_payload_over_the_limit() {
        let payload = [0_u8; MAX_LC3_FRAME_BYTES + 1];
        assert_eq!(
            encode(example_meta(), &payload).err(),
            Some(EncodeError::PayloadTooLong(payload.len()))
        );
    }

    #[test]
    fn packed_pcm_should_round_trip_at_its_maximum_size() {
        let mut meta = example_meta();
        meta.encoding = AudioEncoding::PcmS12LeMono;
        let mut payload = [0_u8; PCM_S12_MONO_FRAME_BYTES];
        for (index, byte) in payload.iter_mut().enumerate() {
            *byte = u8::try_from(index % 256).expect("remainder fits in u8");
        }

        let encoded = encode(meta, &payload).expect("PCM frame must encode");
        let mut parser = StreamDecoder::new();
        let mut completed_frame = None;
        for &byte in encoded.as_bytes() {
            completed_frame = parser.push(byte).expect("PCM frame must decode").or(completed_frame);
        }

        let completed_frame = completed_frame.expect("delimiter must complete the PCM frame");
        assert_eq!(completed_frame.meta, meta);
        assert_eq!(completed_frame.payload(), payload);
    }

    #[test]
    fn pcm_pack_should_preserve_the_top_twelve_bits() {
        let mut input = [0_i16; PCM_MONO_SAMPLES_PER_FRAME];
        for (index, sample) in input.iter_mut().enumerate() {
            *sample = i16::try_from(index)
                .expect("test index fits in i16")
                .wrapping_mul(271)
                .wrapping_sub(31_000);
        }

        let packed = pack_pcm_s12_le(&input).expect("fixed PCM frame must pack");
        let output = unpack_pcm_s12_le(&packed).expect("fixed PCM frame must unpack");

        for (original, restored) in input.into_iter().zip(output) {
            assert_eq!(restored, original & !0x0f);
        }
    }
}
