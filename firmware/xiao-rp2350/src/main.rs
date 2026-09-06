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
use embassy_rp::peripherals::{UART0, USB};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::host::UsbHostBusExt;
use embassy_usb_driver::Speed;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use crate::audio::AudioProgress;
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
const _: () = assert!(PCM_HIGH_DEADBAND_BLOCKS < audio::PCM_RING_CAPACITY);

static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();
static EXECUTOR_CORE0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR_CORE1: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    USBCTRL_IRQ => embassy_rp::usb::host::InterruptHandler<USB>;
});

struct PcmReader {
    receiver: PcmReceiver,
    frame_index: usize,
    buffering: bool,
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

#[derive(Clone, Copy)]
enum BridgeState {
    WaitingForDac,
    DacConfigured,
    UartBytes,
    WireFrame,
    Lc3Decoded,
    Streaming,
}

struct StatusLed {
    user: Output<'static>,
}

impl StatusLed {
    fn show(&mut self, state: BridgeState) {
        if matches!(state, BridgeState::Streaming) {
            self.user.set_low();
        } else {
            self.user.set_high();
        }
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
    // Visible without a probe: one startup flash, three after a watchdog timeout.
    for _ in 0..if watchdog_reset { 3 } else { 1 } {
        led.user.set_low();
        watchdog::idle_wait(watchdog::Task::Usb, Timer::after_millis(120)).await;
        led.user.set_high();
        watchdog::idle_wait(watchdog::Task::Usb, Timer::after_millis(120)).await;
    }
    led.show(BridgeState::WaitingForDac);
    let mut pcm = PcmReader::new(pcm_receiver);
    loop {
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
        led.show(BridgeState::DacConfigured);

        let mut rate = FeedbackRate::new(playback.maximum_packet_frames());
        let mut usb_frames = 0_u32;
        let feedback_interval = playback.feedback_interval_frames();
        let mut feedback_failures = 0_u8;
        let mut feedback_enabled = feedback_interval != 0;
        let mut logged_feedback = false;
        let mut displayed_pcm_blocks = 0_u32;
        let mut cadence_started = Instant::now();
        let mut cadence_pcm_frames = 0_u32;
        let mut transaction_failures = 0_u32;
        let mut applied_volume = None;
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
                led.show(BridgeState::WaitingForDac);
                break;
            }
            let stereo_frames = rate.next_packet_frames();
            let packet_len = stereo_frames * 4;
            if let Err(error) = playback.write_packet_with(packet_len, |packet| {
                pcm.fill_packet(stereo_frames, packet);
                // A bridge reset must not resume an existing BLE stream at default/full gain.
                if applied_volume.is_none() {
                    packet.fill(0);
                }
            }) {
                transaction_failures = transaction_failures.wrapping_add(1);
                if !playback.is_device_connected() {
                    defmt::info!("USB Audio device disconnected during write: {:?}", error);
                    led.show(BridgeState::WaitingForDac);
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
            if pcm.is_buffering() {
                // The active-low yellow LED now goes dark as soon as playback starves instead of
                // remaining latched in the last successful streaming state.
                led.show(BridgeState::Lc3Decoded);
            } else if pcm.received_blocks != displayed_pcm_blocks {
                displayed_pcm_blocks = pcm.received_blocks;
                led.show(BridgeState::Streaming);
            } else if displayed_pcm_blocks == 0 {
                let state = match audio::progress() {
                    AudioProgress::WaitingForUart => BridgeState::DacConfigured,
                    AudioProgress::UartBytes => BridgeState::UartBytes,
                    AudioProgress::WireFrame => BridgeState::WireFrame,
                    AudioProgress::Lc3Decoded => BridgeState::Lc3Decoded,
                    AudioProgress::StereoQueued => BridgeState::Streaming,
                };
                led.show(state);
            }
            usb_frames = usb_frames.wrapping_add(1);

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
    let led = StatusLed {
        user: Output::new(p.PIN_25, Level::High),
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
