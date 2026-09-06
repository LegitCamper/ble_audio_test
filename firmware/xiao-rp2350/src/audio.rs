//! UART ingest, native Rust stereo LC3 decoding, and the zero-copy PCM ring for core 1.

use core::sync::atomic::Ordering;

use ble_audio_link::{
    AudioEncoding, Channel, DecodedFrame, FrameDuration as LinkFrameDuration, FrameFlags, StreamDecoder, VolumeCommand,
};
use embassy_rp::spinlock_mutex::SpinlockRawMutex;
use embassy_rp::uart::BufferedUartRx;
use embassy_sync::zerocopy_channel::{Channel as ZeroCopyChannel, Receiver, Sender};
use embassy_time::Instant;
use embedded_io_async::Read;
use lc3_codec::common::complex::{Complex, Scaler};
use lc3_codec::common::config::{FrameDuration as CodecFrameDuration, SamplingFrequency};
use lc3_codec::decoder::lc3_decoder::Lc3Decoder;
use portable_atomic::{AtomicU8, AtomicU16};
use static_cell::StaticCell;

/// Sample rate of the received LC3 streams and decoded PCM.
pub const SAMPLE_RATE_HZ: u32 = 48_000;
/// PCM samples per channel in one 10 ms frame.
pub const SAMPLES_PER_CHANNEL: usize = (SAMPLE_RATE_HZ / 100) as usize;
/// Number of decoded blocks retained to absorb BLE, decode, and USB timing jitter.
pub const PCM_RING_CAPACITY: usize = 16;

const DECODER_CHANNELS: usize = 2;
const MAX_CONCEALED_BLOCKS_PER_GAP: u16 = 4;
const FRAME_DURATION_US: u32 = 10_000;
const CODEC_FRAME_DURATION: CodecFrameDuration = CodecFrameDuration::TenMs;
const CODEC_SAMPLE_RATE: SamplingFrequency = SamplingFrequency::Hz48000;
const DECODER_BUFFER_LENGTHS: (usize, usize) =
    Lc3Decoder::<DECODER_CHANNELS>::calc_working_buffer_lengths(CODEC_FRAME_DURATION, CODEC_SAMPLE_RATE);

static AUDIO_PROGRESS: AtomicU8 = AtomicU8::new(AudioProgress::WaitingForUart as u8);
/// Latest volume published by core 1 for core 0 to apply, or [`NO_VOLUME`] if the source has not
/// sent one. Packing it into a single word keeps level and mute inseparable across cores, so the
/// USB task can never observe a new level with a stale mute.
static VOLUME: AtomicU16 = AtomicU16::new(NO_VOLUME);
/// Sentinel meaning no volume has been received; distinguishable from level 0 with mute clear.
const NO_VOLUME: u16 = 0xffff;
const VOLUME_MUTED_BIT: u16 = 1 << 8;
static DECODER_SCALER_MEMORY: StaticCell<[Scaler; DECODER_BUFFER_LENGTHS.0]> = StaticCell::new();
static DECODER_COMPLEX_MEMORY: StaticCell<[Complex; DECODER_BUFFER_LENGTHS.1]> = StaticCell::new();

/// Furthest audio-pipeline stage reached since boot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AudioProgress {
    /// No UART bytes have arrived.
    WaitingForUart = 0,
    /// UART bytes arrived, but no complete valid link frame was parsed.
    UartBytes = 1,
    /// A valid framed audio packet was parsed.
    WireFrame = 2,
    /// A matching LC3 stereo pair was decoded.
    Lc3Decoded = 3,
    /// A decoded stereo block was published to USB.
    StereoQueued = 4,
}

/// Returns the furthest audio-pipeline stage reached since boot.
#[must_use]
pub fn progress() -> AudioProgress {
    match AUDIO_PROGRESS.load(Ordering::Relaxed) {
        1 => AudioProgress::UartBytes,
        2 => AudioProgress::WireFrame,
        3 => AudioProgress::Lc3Decoded,
        4 => AudioProgress::StereoQueued,
        _ => AudioProgress::WaitingForUart,
    }
}

fn mark_progress(progress: AudioProgress) {
    AUDIO_PROGRESS.fetch_max(progress as u8, Ordering::Relaxed);
}

/// Publishes a volume received from the LE Audio source for the USB task to apply.
fn publish_volume(command: VolumeCommand) {
    let packed = u16::from(command.level) | if command.muted { VOLUME_MUTED_BIT } else { 0 };
    VOLUME.store(packed, Ordering::Relaxed);
}

/// Takes the pending volume, if the source has sent one since the last call.
///
/// Consuming the value means the USB task only issues a control transfer when something actually
/// changed, rather than on every packet.
pub fn take_volume() -> Option<VolumeCommand> {
    let packed = VOLUME.swap(NO_VOLUME, Ordering::Relaxed);
    if packed == NO_VOLUME {
        return None;
    }
    Some(VolumeCommand {
        level: packed as u8,
        muted: packed & VOLUME_MUTED_BIT != 0,
    })
}

/// One 10 ms block of signed stereo PCM stored as decoder-native channel planes.
pub struct PcmBlock {
    /// Left-channel samples.
    pub left: [i16; SAMPLES_PER_CHANNEL],
    /// Right-channel samples.
    pub right: [i16; SAMPLES_PER_CHANNEL],
    /// Sequence shared by the source left and right frames.
    pub sequence: u16,
    /// nRF receive timestamp shared by the source pair.
    pub timestamp_us: u32,
}

impl PcmBlock {
    const fn empty() -> Self {
        Self {
            left: [0; SAMPLES_PER_CHANNEL],
            right: [0; SAMPLES_PER_CHANNEL],
            sequence: 0,
            timestamp_us: 0,
        }
    }
}

/// Producer half of the decoded PCM ring, owned by core 1.
pub type PcmSender = Sender<'static, SpinlockRawMutex<0>, PcmBlock>;
/// Consumer half of the decoded PCM ring, owned by core 0.
pub type PcmReceiver = Receiver<'static, SpinlockRawMutex<0>, PcmBlock>;
type PcmChannel = ZeroCopyChannel<'static, SpinlockRawMutex<0>, PcmBlock>;

/// Creates the single producer and consumer for the statically allocated PCM ring.
pub fn init_pcm_channel() -> (PcmSender, PcmReceiver) {
    static PCM_BLOCKS: StaticCell<[PcmBlock; PCM_RING_CAPACITY]> = StaticCell::new();
    static PCM_CHANNEL: StaticCell<PcmChannel> = StaticCell::new();

    let blocks = PCM_BLOCKS.init([const { PcmBlock::empty() }; PCM_RING_CAPACITY]);
    PCM_CHANNEL.init(ZeroCopyChannel::new(blocks)).split()
}

#[derive(Default)]
struct StereoPairer {
    left: Option<DecodedFrame>,
    right: Option<DecodedFrame>,
    discarded: u32,
}

impl StereoPairer {
    fn push(&mut self, frame: DecodedFrame) -> Option<(DecodedFrame, DecodedFrame)> {
        match frame.meta.channel {
            Channel::Left => {
                self.discarded = self
                    .discarded
                    .wrapping_add(u32::from(self.left.replace(frame).is_some()));
            }
            Channel::Right => {
                self.discarded = self
                    .discarded
                    .wrapping_add(u32::from(self.right.replace(frame).is_some()));
            }
            Channel::Mono | Channel::Unknown => return None,
        }

        let (Some(left), Some(right)) = (&self.left, &self.right) else {
            return None;
        };
        if left.meta.sequence == right.meta.sequence {
            return Some((self.left.take()?, self.right.take()?));
        }

        self.discarded = self.discarded.wrapping_add(1);
        if sequence_is_after(left.meta.sequence, right.meta.sequence) {
            self.right = None;
        } else {
            self.left = None;
        }
        None
    }
}

fn sequence_is_after(sequence: u16, reference: u16) -> bool {
    let distance = sequence.wrapping_sub(reference);
    distance != 0 && distance < 0x8000
}

/// Runs the core-1 UART parser and decodes matching left/right frames directly into PCM ring slots.
pub async fn run(mut uart: BufferedUartRx, mut pcm: PcmSender) -> ! {
    let scaler_memory = DECODER_SCALER_MEMORY.init_with(|| [0.0; DECODER_BUFFER_LENGTHS.0]);
    let complex_memory = DECODER_COMPLEX_MEMORY.init_with(|| [Complex { r: 0.0, i: 0.0 }; DECODER_BUFFER_LENGTHS.1]);
    let mut decoder =
        Lc3Decoder::<DECODER_CHANNELS>::new(CODEC_FRAME_DURATION, CODEC_SAMPLE_RATE, scaler_memory, complex_memory);
    defmt::info!(
        "native Rust LC3 decoder ready: channels={} scaler_words={} complex_words={}",
        DECODER_CHANNELS,
        DECODER_BUFFER_LENGTHS.0,
        DECODER_BUFFER_LENGTHS.1
    );
    let mut parser = StreamDecoder::new();
    let mut pairer = StereoPairer::default();
    let mut rx_buffer = [0_u8; 128];
    let mut decoded_channels = 0_u32;
    let mut stereo_blocks = 0_u32;
    let mut dropped_blocks = 0_u32;
    let mut failed_channels = 0_u32;
    let mut concealed_blocks = 0_u32;
    let mut unconcealed_blocks = 0_u32;
    let mut rejected_wire_frames = 0_u32;
    let mut maximum_stereo_decode_us = 0_u64;
    let mut maximum_stereo_plc_us = 0_u64;
    let mut last_decoded_sequence = None;
    let mut last_decoded_timestamp_us = 0_u32;
    let mut report_started = Instant::now();

    loop {
        let received = match uart.read(&mut rx_buffer).await {
            Ok(count) => count,
            Err(error) => {
                defmt::warn!("UART receive error: {:?}", error);
                continue;
            }
        };
        if received != 0 {
            mark_progress(AudioProgress::UartBytes);
        }

        for &byte in &rx_buffer[..received] {
            let frame = match parser.push(byte) {
                Ok(Some(frame)) => frame,
                Ok(None) => continue,
                Err(_) => {
                    rejected_wire_frames = rejected_wire_frames.wrapping_add(1);
                    continue;
                }
            };
            mark_progress(AudioProgress::WireFrame);

            if frame.meta.encoding == AudioEncoding::Volume {
                match VolumeCommand::from_payload(frame.payload()) {
                    Some(command) => {
                        defmt::info!("volume received: level={} muted={}", command.level, command.muted);
                        publish_volume(command);
                    }
                    None => defmt::warn!("malformed volume frame: {} bytes", frame.payload().len()),
                }
                continue;
            }

            if frame.meta.encoding != AudioEncoding::Lc3
                || frame.meta.sample_rate_hz != SAMPLE_RATE_HZ
                || frame.meta.frame_duration != LinkFrameDuration::Millis10
                || frame.payload().is_empty()
            {
                defmt::warn!(
                    "unsupported audio config: encoding={} rate={} duration={} bytes={}",
                    frame.meta.encoding as u8,
                    frame.meta.sample_rate_hz,
                    frame.meta.frame_duration as u8,
                    frame.payload().len()
                );
                continue;
            }

            let Some((left, right)) = pairer.push(frame) else {
                continue;
            };
            let sequence = left.meta.sequence;
            let starts_stream = left.meta.flags.contains(FrameFlags::STREAM_START);
            let missing_blocks = if starts_stream {
                0
            } else {
                last_decoded_sequence.map_or(0, |previous: u16| {
                    let distance = sequence.wrapping_sub(previous).wrapping_sub(1);
                    if distance < 0x8000 { distance } else { 0 }
                })
            };
            let conceal_blocks = missing_blocks.min(MAX_CONCEALED_BLOCKS_PER_GAP);
            unconcealed_blocks = unconcealed_blocks.wrapping_add(u32::from(missing_blocks - conceal_blocks));
            let concealment_base_sequence = last_decoded_sequence.unwrap_or(sequence.wrapping_sub(missing_blocks + 1));
            let concealment_base_timestamp_us = last_decoded_timestamp_us;

            for offset in 1..=conceal_blocks {
                let Some(block) = pcm.try_send() else {
                    dropped_blocks = dropped_blocks.wrapping_add(1);
                    break;
                };
                let started = Instant::now();
                let left_status = decoder.decode_frame(16, 0, &[], &mut block.left);
                let right_status = decoder.decode_frame(16, 1, &[], &mut block.right);
                maximum_stereo_plc_us = maximum_stereo_plc_us.max((Instant::now() - started).as_micros());
                if left_status.is_err() || right_status.is_err() {
                    failed_channels = failed_channels
                        .wrapping_add(u32::from(left_status.is_err()))
                        .wrapping_add(u32::from(right_status.is_err()));
                    break;
                }

                let concealed_sequence = concealment_base_sequence.wrapping_add(offset);
                let concealed_timestamp_us =
                    concealment_base_timestamp_us.wrapping_add(u32::from(offset) * FRAME_DURATION_US);
                block.sequence = concealed_sequence;
                block.timestamp_us = concealed_timestamp_us;
                pcm.send_done();
                last_decoded_sequence = Some(concealed_sequence);
                last_decoded_timestamp_us = concealed_timestamp_us;
                concealed_blocks = concealed_blocks.wrapping_add(1);
                mark_progress(AudioProgress::StereoQueued);
            }

            let Some(block) = pcm.try_send() else {
                dropped_blocks = dropped_blocks.wrapping_add(1);
                continue;
            };

            let started = Instant::now();
            let left_status = decoder.decode_frame(16, 0, left.payload(), &mut block.left);
            let right_status = decoder.decode_frame(16, 1, right.payload(), &mut block.right);
            maximum_stereo_decode_us = maximum_stereo_decode_us.max((Instant::now() - started).as_micros());

            if left_status.is_err() || right_status.is_err() {
                failed_channels = failed_channels
                    .wrapping_add(u32::from(left_status.is_err()))
                    .wrapping_add(u32::from(right_status.is_err()));
                defmt::warn!(
                    "native Rust LC3 stereo decode failed for sequence {}",
                    left.meta.sequence
                );
                continue;
            }
            decoded_channels = decoded_channels.wrapping_add(2);
            block.sequence = left.meta.sequence;
            block.timestamp_us = left.meta.timestamp_us;
            pcm.send_done();
            last_decoded_sequence = Some(sequence);
            last_decoded_timestamp_us = left.meta.timestamp_us;

            mark_progress(AudioProgress::Lc3Decoded);
            mark_progress(AudioProgress::StereoQueued);
            stereo_blocks = stereo_blocks.wrapping_add(1);

            if stereo_blocks % 100 == 0 {
                let now = Instant::now();
                let report_us = (now - report_started).as_micros();
                report_started = now;
                defmt::info!(
                    "stereo_blocks={} decoded_lc3={} failed_lc3={} max_stereo_decode_us={} concealed_blocks={} max_stereo_plc_us={} unconcealed_blocks={} dropped_pcm={} unpaired_lc3={} rejected_wire={} last_100_blocks_us={}",
                    stereo_blocks,
                    decoded_channels,
                    failed_channels,
                    maximum_stereo_decode_us,
                    concealed_blocks,
                    maximum_stereo_plc_us,
                    unconcealed_blocks,
                    dropped_blocks,
                    pairer.discarded,
                    rejected_wire_frames,
                    report_us
                );
            }
        }
    }
}
