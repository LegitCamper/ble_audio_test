#![no_std]
#![no_main]

//! Dual-core 48 kHz stereo LC3 decoder and UAC2 USB-host bridge for the XIAO RP2350.

mod audio;
mod iso;
mod usb_audio;
mod watchdog;

use defmt::unwrap;
use embassy_executor::Executor;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::config::Config as RpConfig;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{PIO0, UART0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::Speed;
use smart_leds::RGB8;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::audio::SAMPLES_PER_CHANNEL;
use crate::audio::{PcmReceiver, PcmSender};
use crate::usb_audio::{MINIMUM_PACKET_FRAMES, NOMINAL_PACKET_FRAMES, UsbAudioPlayback, VolumeOwner};

const CORE1_STACK_SIZE: usize = 64 * 1024;
#[cfg(not(feature = "stock-clock"))]
const SYSTEM_CLOCK_HZ: u32 = 300_000_000;
#[cfg(feature = "stock-clock")]
const SYSTEM_CLOCK_HZ: u32 = 150_000_000;
const _: () = assert!(usb_audio::SAMPLE_RATE_HZ == audio::SAMPLE_RATE_HZ);
const PCM_PREBUFFER_BLOCKS: usize = 6;
const PCM_TARGET_BLOCKS: usize = PCM_PREBUFFER_BLOCKS;
const PCM_HIGH_DEADBAND_BLOCKS: usize = PCM_TARGET_BLOCKS + 1;
const PCM_NEAR_CORRECTION_INTERVAL_FRAMES: usize = 1_536;
const PCM_FAR_CORRECTION_INTERVAL_FRAMES: usize = 384;
const PCM_CRITICAL_CORRECTION_INTERVAL_FRAMES: usize = 96;
const RGB_BRIGHTNESS: u8 = 16;
const RECENT_SIGNAL_HOLD_USB_FRAMES: u16 = 2_000;
const PIPELINE_ACTIVITY_HOLD_USB_FRAMES: u16 = 500;
const PIPELINE_UART_ACTIVE: u8 = 1 << 0;
const PIPELINE_LC3_ACTIVE: u8 = 1 << 1;
const PIPELINE_PAIRS_ACTIVE: u8 = 1 << 2;
const PIPELINE_DECODE_ACTIVE: u8 = 1 << 3;
const PIPELINE_ALL_ACTIVE: u8 =
    PIPELINE_UART_ACTIVE | PIPELINE_LC3_ACTIVE | PIPELINE_PAIRS_ACTIVE | PIPELINE_DECODE_ACTIVE;
const _: () = assert!(PCM_HIGH_DEADBAND_BLOCKS < audio::PCM_RING_CAPACITY);
const _: () = assert!(matches!(playback_status(false, false, false, 0, 0), RgbStatus::Off));
const _: () = assert!(matches!(
    playback_status(true, true, true, RECENT_SIGNAL_HOLD_USB_FRAMES, 0),
    RgbStatus::NoUart
));
const _: () = assert!(matches!(
    playback_status(true, true, true, RECENT_SIGNAL_HOLD_USB_FRAMES, PIPELINE_UART_ACTIVE),
    RgbStatus::InvalidFrames
));
const _: () = assert!(matches!(
    playback_status(
        true,
        true,
        true,
        RECENT_SIGNAL_HOLD_USB_FRAMES,
        PIPELINE_UART_ACTIVE | PIPELINE_LC3_ACTIVE
    ),
    RgbStatus::PairOrDecodeFailure
));
const _: () = assert!(matches!(
    playback_status(true, true, true, RECENT_SIGNAL_HOLD_USB_FRAMES, PIPELINE_ALL_ACTIVE),
    RgbStatus::Rebuffering
));
const _: () = assert!(matches!(
    playback_status(false, true, false, RECENT_SIGNAL_HOLD_USB_FRAMES, 0),
    RgbStatus::OutputMuted
));
const _: () = assert!(matches!(
    playback_status(false, true, true, 0, 0),
    RgbStatus::DigitalSilence
));
const _: () = assert!(matches!(
    playback_status(false, true, true, RECENT_SIGNAL_HOLD_USB_FRAMES, 0),
    RgbStatus::Streaming
));

static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();
static EXECUTOR_CORE0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_CORE1: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    USBCTRL_IRQ => embassy_rp::usb::host::InterruptHandler<USB>;
});

struct PcmReader {
    receiver: PcmReceiver,
    frame_index: usize,
    buffering: bool,
    playback_started: bool,
    last_frame: Option<(i16, i16)>,
    correction_cooldown_frames: usize,
    underflows: u32,
    inserted_frames: u32,
    discarded_frames: u32,
    missing_source_blocks: u32,
    last_left_sequence: u16,
    last_right_sequence: u16,
    last_timestamp_us: u32,
    received_blocks: u32,
}

impl PcmReader {
    fn new(receiver: PcmReceiver) -> Self {
        Self {
            receiver,
            frame_index: 0,
            buffering: true,
            playback_started: false,
            last_frame: None,
            correction_cooldown_frames: 0,
            underflows: 0,
            inserted_frames: 0,
            discarded_frames: 0,
            missing_source_blocks: 0,
            last_left_sequence: 0,
            last_right_sequence: 0,
            last_timestamp_us: 0,
            received_blocks: 0,
        }
    }

    const fn is_buffering(&self) -> bool {
        self.buffering
    }

    fn buffered_blocks(&self) -> usize {
        self.receiver.len()
    }

    fn release_finished_block(&mut self) {
        if self.frame_index == SAMPLES_PER_CHANNEL {
            self.receiver.receive_done();
            self.frame_index = 0;
        }
    }

    fn start_source_block(&mut self, sequence: u16, timestamp_us: u32) {
        if self.received_blocks != 0 {
            let missing = sequence.wrapping_sub(self.last_left_sequence).wrapping_sub(1);
            if missing < 0x8000 {
                self.missing_source_blocks = self.missing_source_blocks.wrapping_add(u32::from(missing));
            }
        }
        self.last_left_sequence = sequence;
        self.last_right_sequence = sequence;
        self.last_timestamp_us = timestamp_us;
        self.received_blocks = self.received_blocks.wrapping_add(1);
    }

    fn peek_source_frame(&mut self) -> Option<(i16, i16)> {
        self.release_finished_block();
        let block = self.receiver.try_receive()?;
        Some((block.left[self.frame_index], block.right[self.frame_index]))
    }

    fn discard_source_frame(&mut self) -> Option<(i16, i16)> {
        self.release_finished_block();
        let block = self.receiver.try_receive()?;
        let block_metadata = (self.frame_index == 0).then_some((block.sequence, block.timestamp_us));
        let discarded = (block.left[self.frame_index], block.right[self.frame_index]);
        self.frame_index += 1;
        if let Some((sequence, timestamp_us)) = block_metadata {
            self.start_source_block(sequence, timestamp_us);
        }
        self.release_finished_block();
        Some(discarded)
    }

    fn fill_packet(&mut self, stereo_frames: usize, output: &mut [u8]) {
        debug_assert_eq!(output.len(), stereo_frames * 4);
        self.correction_cooldown_frames = self.correction_cooldown_frames.saturating_sub(stereo_frames);
        if self.buffering {
            if self.receiver.len() < PCM_PREBUFFER_BLOCKS {
                self.underflows = self.underflows.wrapping_add(1);
                output.fill(0);
                return;
            }
            self.buffering = false;
            self.playback_started = true;
        }

        let mut output_frames = output.chunks_exact_mut(4);
        let mut discarded_frame = None;
        if self.correction_cooldown_frames == 0 {
            let buffered_blocks = self.receiver.len();
            if buffered_blocks < PCM_TARGET_BLOCKS {
                if let Some(previous) = self.last_frame
                    && let Some(next) = self.peek_source_frame()
                    && let Some(frame) = output_frames.next()
                {
                    let inserted = midpoint_stereo_frame(previous, next);
                    write_stereo_frame(frame, inserted.0, inserted.1);
                    self.last_frame = Some(inserted);
                    self.inserted_frames = self.inserted_frames.wrapping_add(1);
                    self.correction_cooldown_frames = correction_interval(PCM_TARGET_BLOCKS - buffered_blocks);
                }
            } else if buffered_blocks > PCM_HIGH_DEADBAND_BLOCKS {
                discarded_frame = self.discard_source_frame();
                if discarded_frame.is_some() {
                    self.discarded_frames = self.discarded_frames.wrapping_add(1);
                    self.correction_cooldown_frames = correction_interval(buffered_blocks - PCM_HIGH_DEADBAND_BLOCKS);
                }
            }
        }

        while output_frames.len() != 0 {
            self.release_finished_block();
            let Some(block) = self.receiver.try_receive() else {
                self.underflows = self.underflows.wrapping_add(1);
                self.buffering = true;
                defmt::warn!("PCM underflow; rebuffering, count={}", self.underflows);
                output_frames.for_each(|remaining| remaining.fill(0));
                return;
            };
            let block_metadata = (self.frame_index == 0).then_some((block.sequence, block.timestamp_us));

            let copied_frames = output_frames.len().min(SAMPLES_PER_CHANNEL - self.frame_index);
            let source_range = self.frame_index..self.frame_index + copied_frames;
            let mut last_frame = None;
            for (frame, (&left, &right)) in output_frames
                .by_ref()
                .take(copied_frames)
                .zip(block.left[source_range.clone()].iter().zip(&block.right[source_range]))
            {
                let output = discarded_frame.take().map_or((left, right), |discarded| {
                    midpoint_stereo_frame(discarded, (left, right))
                });
                write_stereo_frame(frame, output.0, output.1);
                last_frame = Some(output);
            }
            self.frame_index += copied_frames;
            self.last_frame = last_frame;
            if let Some((sequence, timestamp_us)) = block_metadata {
                self.start_source_block(sequence, timestamp_us);
            }
        }
        self.release_finished_block();
    }
}

const fn correction_interval(distance_blocks: usize) -> usize {
    match distance_blocks {
        0 | 1 => PCM_NEAR_CORRECTION_INTERVAL_FRAMES,
        2 => PCM_FAR_CORRECTION_INTERVAL_FRAMES,
        _ => PCM_CRITICAL_CORRECTION_INTERVAL_FRAMES,
    }
}

fn midpoint_stereo_frame(first: (i16, i16), second: (i16, i16)) -> (i16, i16) {
    let midpoint = |a, b| ((i32::from(a) + i32::from(b)) / 2) as i16;
    (midpoint(first.0, second.0), midpoint(first.1, second.1))
}

fn write_stereo_frame(output: &mut [u8], left: i16, right: i16) {
    output[..2].copy_from_slice(&left.to_le_bytes());
    output[2..].copy_from_slice(&right.to_le_bytes());
}

#[derive(Clone, Copy, Eq, PartialEq)]
enum RgbStatus {
    Off,
    WatchdogReset,
    Streaming,
    Rebuffering,
    NoUart,
    InvalidFrames,
    PairOrDecodeFailure,
    DigitalSilence,
    OutputMuted,
}

const fn playback_status(
    buffering: bool,
    playback_started: bool,
    volume_applied: bool,
    recent_signal_frames: u16,
    pipeline_activity: u8,
) -> RgbStatus {
    if !playback_started {
        RgbStatus::Off
    } else if buffering {
        if pipeline_activity & PIPELINE_UART_ACTIVE == 0 {
            RgbStatus::NoUart
        } else if pipeline_activity & PIPELINE_LC3_ACTIVE == 0 {
            RgbStatus::InvalidFrames
        } else if pipeline_activity & (PIPELINE_PAIRS_ACTIVE | PIPELINE_DECODE_ACTIVE)
            != PIPELINE_PAIRS_ACTIVE | PIPELINE_DECODE_ACTIVE
        {
            RgbStatus::PairOrDecodeFailure
        } else {
            RgbStatus::Rebuffering
        }
    } else if !volume_applied {
        RgbStatus::OutputMuted
    } else if recent_signal_frames == 0 {
        RgbStatus::DigitalSilence
    } else {
        RgbStatus::Streaming
    }
}

fn packet_has_signal(packet: &[u8]) -> bool {
    packet.chunks_exact(2).any(|sample| sample != [0, 0])
}

fn update_activity(counter: u32, previous: &mut u32, remaining_frames: &mut u16) {
    if counter != *previous {
        *previous = counter;
        *remaining_frames = PIPELINE_ACTIVITY_HOLD_USB_FRAMES;
    } else {
        *remaining_frames = remaining_frames.saturating_sub(1);
    }
}

type RgbDriver = PioWs2812<'static, PIO0, 0, 1, Grb>;

struct StatusLed {
    rgb: RgbDriver,
    power: Output<'static>,
    current: Option<RgbStatus>,
}

impl StatusLed {
    fn enable(&mut self) {
        self.power.set_high();
    }

    async fn show(&mut self, status: RgbStatus) {
        if self.current == Some(status) {
            return;
        }
        let color = match status {
            RgbStatus::Off => RGB8::new(0, 0, 0),
            RgbStatus::WatchdogReset => RGB8::new(RGB_BRIGHTNESS, 0, 0),
            RgbStatus::Streaming => RGB8::new(0, RGB_BRIGHTNESS, 0),
            RgbStatus::Rebuffering => RGB8::new(RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0),
            RgbStatus::NoUart => RGB8::new(0, 0, RGB_BRIGHTNESS),
            RgbStatus::InvalidFrames => RGB8::new(RGB_BRIGHTNESS, RGB_BRIGHTNESS / 4, 0),
            RgbStatus::PairOrDecodeFailure => RGB8::new(RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS / 2),
            RgbStatus::DigitalSilence => RGB8::new(0, RGB_BRIGHTNESS, RGB_BRIGHTNESS),
            RgbStatus::OutputMuted => RGB8::new(RGB_BRIGHTNESS, 0, RGB_BRIGHTNESS),
        };
        self.rgb.write(&[color]).await;
        self.current = Some(status);
    }
}

struct FeedbackRate {
    samples_per_frame_q16: u32,
    phase_q16: u32,
    maximum_packet_frames: u32,
}

impl FeedbackRate {
    const fn new(maximum_packet_frames: u8) -> Self {
        Self {
            samples_per_frame_q16: NOMINAL_PACKET_FRAMES << 16,
            phase_q16: 0,
            maximum_packet_frames: maximum_packet_frames as u32,
        }
    }

    fn update(&mut self, feedback_q16: u32) {
        let integer = feedback_q16 >> 16;
        if (MINIMUM_PACKET_FRAMES..=self.maximum_packet_frames).contains(&integer) {
            self.samples_per_frame_q16 = feedback_q16;
        }
    }

    fn next_packet_frames(&mut self) -> usize {
        self.phase_q16 = self.phase_q16.wrapping_add(self.samples_per_frame_q16);
        let frames = self.phase_q16 >> 16;
        self.phase_q16 &= 0xffff;
        usize::try_from(frames.clamp(MINIMUM_PACKET_FRAMES, self.maximum_packet_frames))
            .unwrap_or(NOMINAL_PACKET_FRAMES as usize)
    }
}

#[embassy_executor::task]
async fn audio_ingest_task(uart: BufferedUartRx, pcm: PcmSender) -> ! {
    audio::run(uart, pcm).await
}

fn attached_speed() -> Option<Speed> {
    match embassy_rp::pac::USB.sie_status().read().speed() {
        0b01 => Some(Speed::Low),
        0b10 => Some(Speed::Full),
        _ => None,
    }
}

async fn wait_for_dac() -> Speed {
    loop {
        if let Some(speed) = attached_speed() {
            return speed;
        }
        Timer::after_millis(20).await;
    }
}

async fn wait_for_dac_removal() {
    while attached_speed().is_some() {
        Timer::after_millis(20).await;
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut host: embassy_rp::usb::host::Driver<'static, USB>,
    mut led: StatusLed,
    pcm_receiver: PcmReceiver,
    watchdog_reset: bool,
) -> ! {
    led.enable();
    led.show(RgbStatus::Off).await;
    if watchdog_reset {
        led.show(RgbStatus::WatchdogReset).await;
        watchdog::idle_wait(watchdog::Task::Usb, Timer::after_secs(1)).await;
        led.show(RgbStatus::Off).await;
    }
    let mut pcm = PcmReader::new(pcm_receiver);
    loop {
        led.show(RgbStatus::Off).await;
        // The pinned driver's event wait only observes edges. Read the port level so
        // a DAC present during startup flashes or a watchdog reset is enumerated too.
        let speed = watchdog::idle_wait(watchdog::Task::Usb, wait_for_dac()).await;
        defmt::info!("USB device connected at {:?}", speed);

        let enumeration = match host.enumerate_root_bare(speed, 1).await {
            Ok(info) => info,
            Err(error) => {
                defmt::warn!("USB enumeration failed: {:?}", error);
                continue;
            }
        };
        let mut playback = match UsbAudioPlayback::configure(&host, &enumeration).await {
            Ok(playback) => playback,
            Err(error) => {
                defmt::warn!("USB Audio setup failed: {:?}", error);
                watchdog::idle_wait(watchdog::Task::Usb, wait_for_dac_removal()).await;
                continue;
            }
        };

        let mut rate = FeedbackRate::new(playback.maximum_packet_frames());
        let mut usb_frames = 0_u32;
        let feedback_interval = playback.feedback_interval_frames();
        let mut feedback_failures = 0_u8;
        let mut feedback_enabled = feedback_interval != 0;
        let mut logged_feedback = false;
        let mut cadence_started = Instant::now();
        let mut cadence_pcm_frames = 0_u32;
        let mut transaction_failures = 0_u32;
        let mut applied_volume = None;
        let mut recent_signal_frames = 0_u16;
        let mut pipeline = audio::pipeline_counters();
        let mut uart_activity_frames = 0_u16;
        let mut lc3_activity_frames = 0_u16;
        let mut pair_activity_frames = 0_u16;
        let mut decode_activity_frames = 0_u16;
        let mut volume_retry_after = Instant::now();
        defmt::info!(
            "USB Audio playback started: rate={} feedback_interval_frames={} volume_owner={}",
            usb_audio::SAMPLE_RATE_HZ,
            feedback_interval,
            match playback.volume_owner() {
                VolumeOwner::Dac => "dac",
                VolumeOwner::Host => "rp2350",
            }
        );

        loop {
            watchdog::progress(watchdog::Task::Usb);
            if let Err(error) = playback.wait_for_sof().await {
                defmt::info!("USB Audio device left the bus: {:?}", error);
                led.show(RgbStatus::Off).await;
                break;
            }
            let stereo_frames = rate.next_packet_frames();
            let packet_len = stereo_frames * 4;
            let mut packet_has_pcm_signal = false;
            if let Err(error) = playback.write_packet_with(packet_len, |packet| {
                pcm.fill_packet(stereo_frames, packet);
                packet_has_pcm_signal = packet_has_signal(packet);
                // A bridge reset must not resume an existing BLE stream at default/full gain.
                if applied_volume.is_none() {
                    packet.fill(0);
                }
            }) {
                transaction_failures = transaction_failures.wrapping_add(1);
                if !playback.is_device_connected() {
                    defmt::info!("USB Audio device disconnected during write: {:?}", error);
                    led.show(RgbStatus::Off).await;
                    break;
                }
                if transaction_failures == 1 || transaction_failures % 100 == 0 {
                    defmt::warn!(
                        "transient isochronous write failure; continuing: error={:?} count={}",
                        error,
                        transaction_failures
                    );
                }
                continue;
            }
            cadence_pcm_frames = cadence_pcm_frames.wrapping_add(stereo_frames as u32);
            usb_frames = usb_frames.wrapping_add(1);
            recent_signal_frames = if packet_has_pcm_signal {
                RECENT_SIGNAL_HOLD_USB_FRAMES
            } else {
                recent_signal_frames.saturating_sub(1)
            };
            let current_pipeline = audio::pipeline_counters();
            update_activity(
                current_pipeline.uart_bytes,
                &mut pipeline.uart_bytes,
                &mut uart_activity_frames,
            );
            update_activity(
                current_pipeline.lc3_wire_frames,
                &mut pipeline.lc3_wire_frames,
                &mut lc3_activity_frames,
            );
            update_activity(
                current_pipeline.stereo_pairs,
                &mut pipeline.stereo_pairs,
                &mut pair_activity_frames,
            );
            update_activity(
                current_pipeline.decoded_blocks,
                &mut pipeline.decoded_blocks,
                &mut decode_activity_frames,
            );
            let pipeline_activity = (u8::from(uart_activity_frames != 0) * PIPELINE_UART_ACTIVE)
                | (u8::from(lc3_activity_frames != 0) * PIPELINE_LC3_ACTIVE)
                | (u8::from(pair_activity_frames != 0) * PIPELINE_PAIRS_ACTIVE)
                | (u8::from(decode_activity_frames != 0) * PIPELINE_DECODE_ACTIVE);
            led.show(playback_status(
                pcm.is_buffering(),
                pcm.playback_started,
                applied_volume.is_some(),
                recent_signal_frames,
                pipeline_activity,
            ))
            .await;

            // Applied straight after the audio write so a control transfer to a DAC-owned volume
            // never delays the isochronous packet. Volume changes are user-paced, so this costs
            // nothing on the overwhelming majority of frames.
            if let Some(command) = audio::latest_volume()
                && applied_volume != Some(command)
                && Instant::now() >= volume_retry_after
            {
                match playback.set_volume(command.level, command.muted).await {
                    Ok(()) => applied_volume = Some(command),
                    Err(error) => {
                        defmt::warn!("volume update failed: {:?}", error);
                        applied_volume = None;
                        volume_retry_after = Instant::now() + Duration::from_secs(1);
                    }
                }
            }

            // Keep the time-critical audio OUT transaction first. A cheap Full-Speed DAC may
            // intermittently omit feedback; repeated failures fall back to the nominal packet
            // size rather than terminating playback.
            if feedback_enabled && usb_frames % feedback_interval == 0 {
                match playback.read_feedback() {
                    Ok(value) => {
                        rate.update(value);
                        feedback_failures = 0;
                        if !logged_feedback {
                            defmt::info!("first USB feedback: q16={} whole_frames_per_ms={}", value, value >> 16);
                            logged_feedback = true;
                        }
                    }
                    Err(error) => {
                        feedback_failures = feedback_failures.saturating_add(1);
                        defmt::warn!("feedback read failed: {:?}", error);
                        if feedback_failures == 8 {
                            feedback_enabled = false;
                            defmt::warn!("feedback disabled; using fixed {}-frame packets", NOMINAL_PACKET_FRAMES);
                        }
                    }
                }
            }
            if usb_frames % 1_000 == 0 {
                let now = Instant::now();
                let cadence_us = (now - cadence_started).as_micros();
                cadence_started = now;
                defmt::info!(
                    "USB packets={} last_1000_packets_us={} sent_pcm_frames={} buffered_blocks={} underflows={} inserted_frames={} discarded_frames={} missing_source_blocks={} left_seq={} right_seq={} source_ts={}",
                    usb_frames,
                    cadence_us,
                    cadence_pcm_frames,
                    pcm.buffered_blocks(),
                    pcm.underflows,
                    pcm.inserted_frames,
                    pcm.discarded_frames,
                    pcm.missing_source_blocks,
                    pcm.last_left_sequence,
                    pcm.last_right_sequence,
                    pcm.last_timestamp_us
                );
                cadence_pcm_frames = 0;
            }
        }
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut clocks = unwrap!(ClockConfig::system_freq(SYSTEM_CLOCK_HZ));
    // Stereo LC3 decoding underflows at 225 MHz. Sustained playback therefore needs the measured
    // decoder headroom at 300 MHz; independent source/DAC clock drift is handled in `PcmReader`.
    clocks.core_voltage = if cfg!(feature = "stock-clock") {
        CoreVoltage::V1_10
    } else {
        CoreVoltage::V1_25
    };
    let p = embassy_rp::init(RpConfig::new(clocks));
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let watchdog_reset = watchdog::start(&mut watchdog);
    defmt::info!("RP2350 system clock={} Hz", embassy_rp::clocks::clk_sys_freq());
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 1_000_000;
    static UART_RX_BUFFER: StaticCell<[u8; 8192]> = StaticCell::new();
    let uart = BufferedUartRx::new(p.UART0, Irqs, p.PIN_1, UART_RX_BUFFER.init([0; 8192]), uart_config);
    let usb = embassy_rp::usb::host::Driver::new(p.USB, Irqs);
    let _user_led = Output::new(p.PIN_25, Level::High);
    let rgb_power = Output::new(p.PIN_23, Level::Low);
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let rgb_program = PioWs2812Program::new(&mut common);
    let rgb = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_22, &rgb_program);
    let led = StatusLed {
        rgb,
        power: rgb_power,
        current: None,
    };
    let (pcm_sender, pcm_receiver) = audio::init_pcm_channel();

    spawn_core1(
        p.CORE1,
        // SAFETY: core 1 is spawned exactly once, and this is the only reference ever created to
        // its static stack storage. `spawn_core1` retains it for the lifetime of that core.
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = EXECUTOR_CORE1.init(Executor::new());
            executor.run(|spawner| spawner.spawn(unwrap!(audio_ingest_task(uart, pcm_sender))));
        },
    );

    let executor = EXECUTOR_CORE0.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(unwrap!(watchdog::run(watchdog)));
        spawner.spawn(unwrap!(usb_task(usb, led, pcm_receiver, watchdog_reset)));
    });
}
