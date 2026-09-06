#![no_std]
#![no_main]

//! nRF54L15 LE Audio sink that forwards paired 48 kHz left/right LC3 frames.

extern crate alloc;

mod audio_sink;
mod bond_store;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use ble_audio_link::{
    AudioEncoding, Channel, FrameDuration as LinkFrameDuration, FrameFlags, FrameMeta, VolumeCommand, encode,
};
use defmt::unwrap;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt;
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::uarte::{self, UarteTx};
use embassy_nrf::{bind_interrupts, config, cracen, mode::Blocking, peripherals};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel as AsyncChannel;
use embassy_time::Instant;
use embedded_alloc::LlffHeap as Heap;
use heapless::Deque;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use static_cell::{ConstStaticCell, StaticCell};
use trouble_audio::cis::{CisManager, Lc3Frame};
use trouble_audio::generic_audio::AudioLocation;
use trouble_audio_example_apps::basic_audio_sink;
use trouble_host::prelude::*;
use {defmt_rtt as _, panic_probe as _};

use crate::bond_store::rram_bond_store;

const HEAP_SIZE: usize = 80 * 1024;
const L2CAP_TXQ: u8 = 3;
const L2CAP_RXQ: u8 = 3;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    SWI00 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO_0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER10 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    GRTC_3 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    SERIAL21 => uarte::InterruptHandler<peripherals::SERIAL21>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

fn build_sdc<'d, const N: usize>(
    peripherals: nrf_sdc::Peripherals<'d>,
    rng: &'d mut cracen::Cracen<'static, Blocking>,
    mpsl: &'d MultiprotocolServiceLayer,
    memory: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()
        .support_ext_adv()
        .support_peripheral()
        .support_cis_peripheral()
        .peripheral_count(1)?
        .buffer_cfg(
            DefaultPacketPool::MTU as u16,
            DefaultPacketPool::MTU as u16,
            L2CAP_TXQ,
            L2CAP_RXQ,
        )?
        .cig_count(1)?
        .cis_count(2)?
        .iso_buffer_cfg(4, 128, 4, 6, 8, 128)?
        .build(peripherals, rng, mpsl, memory)
}

const PENDING_FRAMES_PER_CHANNEL: usize = 8;
const MAXIMUM_INITIAL_ALIGNMENT_SKEW_US: u32 = 5_000;
const STREAM_IDLE_RESET_US: u32 = 100_000;

struct StereoLc3Pair {
    left_ase_id: u8,
    right_ase_id: u8,
    sequence: u16,
    timestamp_us: u32,
    flags: FrameFlags,
    left: Lc3Frame,
    right: Lc3Frame,
}

struct PendingLc3 {
    ase_id: u8,
    sequence: u16,
    timestamp_us: u32,
    receipt_timestamp_us: u32,
    timestamp_from_controller: bool,
    frame: Lc3Frame,
}

#[derive(Default)]
struct StereoPairer {
    left: Deque<PendingLc3, PENDING_FRAMES_PER_CHANNEL>,
    right: Deque<PendingLc3, PENDING_FRAMES_PER_CHANNEL>,
    right_to_left_sequence_offset: Option<u16>,
    last_receipt_timestamp_us: Option<u32>,
    discarded: u32,
}

impl StereoPairer {
    fn push(&mut self, channel: Channel, pending: PendingLc3) -> Option<(PendingLc3, PendingLc3)> {
        if self
            .last_receipt_timestamp_us
            .is_some_and(|last| pending.receipt_timestamp_us.wrapping_sub(last) > STREAM_IDLE_RESET_US)
        {
            self.left.clear();
            self.right.clear();
            self.right_to_left_sequence_offset = None;
            defmt::info!("stereo sequence alignment reset after stream idle");
        }
        self.last_receipt_timestamp_us = Some(pending.receipt_timestamp_us);

        let queue = match channel {
            Channel::Left => &mut self.left,
            Channel::Right => &mut self.right,
            Channel::Mono | Channel::Unknown => return None,
        };
        if let Err(pending) = queue.push_back(pending) {
            let _ = queue.pop_front();
            self.discarded = self.discarded.wrapping_add(1);
            let _ = queue.push_back(pending);
        }

        loop {
            let (Some(left), Some(right)) = (self.left.front(), self.right.front()) else {
                return None;
            };

            let sequence_offset = if let Some(offset) = self.right_to_left_sequence_offset {
                offset
            } else {
                let (left_timestamp, right_timestamp) = alignment_timestamps(left, right);
                let left_after_right = left_timestamp.wrapping_sub(right_timestamp);
                let skew_us = left_after_right.min(right_timestamp.wrapping_sub(left_timestamp));
                if skew_us > MAXIMUM_INITIAL_ALIGNMENT_SKEW_US {
                    self.discarded = self.discarded.wrapping_add(1);
                    if left_after_right < 0x8000_0000 {
                        let _ = self.right.pop_front();
                    } else {
                        let _ = self.left.pop_front();
                    }
                    continue;
                }

                let offset = left.sequence.wrapping_sub(right.sequence);
                self.right_to_left_sequence_offset = Some(offset);
                defmt::info!(
                    "stereo sequence alignment left={} right={} right_offset={} timestamp_skew_us={}",
                    left.sequence,
                    right.sequence,
                    offset,
                    skew_us
                );
                offset
            };

            let aligned_right_sequence = right.sequence.wrapping_add(sequence_offset);
            if left.sequence == aligned_right_sequence {
                return Some((self.left.pop_front()?, self.right.pop_front()?));
            }

            self.discarded = self.discarded.wrapping_add(1);
            if sequence_is_after(left.sequence, aligned_right_sequence) {
                let _ = self.right.pop_front();
            } else {
                let _ = self.left.pop_front();
            }
        }
    }
}

fn alignment_timestamps(left: &PendingLc3, right: &PendingLc3) -> (u32, u32) {
    if left.timestamp_from_controller && right.timestamp_from_controller {
        (left.timestamp_us, right.timestamp_us)
    } else {
        (left.receipt_timestamp_us, right.receipt_timestamp_us)
    }
}

fn sequence_is_after(sequence: u16, reference: u16) -> bool {
    let distance = sequence.wrapping_sub(reference);
    distance != 0 && distance < 0x8000
}

static LC3_TX_QUEUE: AsyncChannel<CriticalSectionRawMutex, StereoLc3Pair, 4> = AsyncChannel::new();
/// Volume settings waiting to be forwarded to the RP2350.
///
/// Kept separate from [`LC3_TX_QUEUE`] and holding the two-byte command rather than an encoded
/// frame, so a volume change neither displaces queued audio nor costs a full wire-frame buffer.
static VOLUME_TX_QUEUE: AsyncChannel<CriticalSectionRawMutex, VolumeCommand, 2> = AsyncChannel::new();
static UART_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI01() {
    // SAFETY: this handler is the sole interrupt entry point for `UART_EXECUTOR`, which is
    // started before SWI01 is enabled.
    unsafe { UART_EXECUTOR.on_interrupt() }
}

/// Queues a volume setting for the RP2350.
///
/// Dropping the setting when the queue is full is safe: only the newest value matters, and the
/// queue only fills if the UART is already wedged.
fn send_volume(command: VolumeCommand) {
    if VOLUME_TX_QUEUE.try_send(command).is_err() {
        defmt::warn!(
            "volume queue full; dropped level={} muted={}",
            command.level,
            command.muted
        );
    }
}

fn channel_from_allocation(allocation: Option<AudioLocation>) -> Option<Channel> {
    let allocation = allocation?;
    let left = allocation.contains(AudioLocation::FrontLeft);
    let right = allocation.contains(AudioLocation::FrontRight);
    match (left, right, allocation.is_empty()) {
        (true, false, _) => Some(Channel::Left),
        (false, true, _) => Some(Channel::Right),
        (false, false, true) => Some(Channel::Mono),
        _ => None,
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart: UarteTx<'static>) -> ! {
    let mut transmitted_pairs = 0_u32;
    let mut volume_sequence = 0_u16;
    loop {
        // Neither branch consumes its item until it is selected, so losing the race cannot drop a
        // queued value.
        match select(LC3_TX_QUEUE.receive(), VOLUME_TX_QUEUE.receive()).await {
            Either::First(pair) => {
                let left_meta = FrameMeta {
                    encoding: AudioEncoding::Lc3,
                    channel: Channel::Left,
                    ase_id: pair.left_ase_id,
                    sequence: pair.sequence,
                    timestamp_us: pair.timestamp_us,
                    sample_rate_hz: 48_000,
                    frame_duration: LinkFrameDuration::Millis10,
                    flags: pair.flags,
                };
                let Ok(encoded) = encode(left_meta, &pair.left) else {
                    defmt::warn!("left LC3 frame exceeded link capacity");
                    continue;
                };
                if uart.write(encoded.as_bytes()).await.is_err() {
                    defmt::warn!("stereo LC3 UART write failed");
                    continue;
                }

                let right_meta = FrameMeta {
                    channel: Channel::Right,
                    ase_id: pair.right_ase_id,
                    ..left_meta
                };
                let Ok(encoded) = encode(right_meta, &pair.right) else {
                    defmt::warn!("right LC3 frame exceeded link capacity");
                    continue;
                };
                if uart.write(encoded.as_bytes()).await.is_err() {
                    defmt::warn!("stereo LC3 UART write failed");
                    continue;
                }
                transmitted_pairs = transmitted_pairs.wrapping_add(1);
                if transmitted_pairs == 1 || transmitted_pairs % 200 == 0 {
                    defmt::info!("transmitted LC3 stereo pairs={}", transmitted_pairs);
                }
            }
            Either::Second(command) => {
                let meta = FrameMeta {
                    encoding: AudioEncoding::Volume,
                    // A volume frame belongs to no stream, so the audio fields carry neutral
                    // values; only the sequence is meaningful, for spotting a dropped update.
                    channel: Channel::Unknown,
                    ase_id: 0,
                    sequence: volume_sequence,
                    timestamp_us: Instant::now().as_micros() as u32,
                    sample_rate_hz: 48_000,
                    frame_duration: LinkFrameDuration::Millis10,
                    flags: FrameFlags::empty(),
                };
                volume_sequence = volume_sequence.wrapping_add(1);
                let Ok(frame) = encode(meta, &command.to_payload()) else {
                    defmt::warn!("could not encode a volume frame");
                    continue;
                };
                if uart.write(frame.as_bytes()).await.is_err() {
                    defmt::warn!("volume UART write failed");
                    continue;
                }
                defmt::info!("sent volume level={} muted={}", command.level, command.muted);
            }
        }
    }
}

async fn forward_lc3(
    led: &mut Output<'static>,
    cis_manager: &CisManager<NoopRawMutex, { basic_audio_sink::MAX_ASES }>,
) -> ! {
    let mut last_sequence: Option<u16> = None;
    let mut last_timestamp_us = 0_u32;
    let mut pairer = StereoPairer::default();
    let mut started = false;
    let mut queue_overflowed = false;
    let mut received_frames = 0_u32;
    let mut stereo_pairs = 0_u32;
    let mut dropped_pairs = 0_u32;
    let mut report_started = Instant::now();

    loop {
        let raw = cis_manager.receive_lc3().await;
        let Some(channel) = channel_from_allocation(raw.channel_allocation) else {
            defmt::warn!("ignored ASE {} without a single left/right allocation", raw.ase_id);
            continue;
        };

        let receipt_micros = Instant::now().as_micros();
        let receipt_timestamp_us = u32::try_from(receipt_micros & u64::from(u32::MAX)).unwrap_or_default();
        let timestamp_from_controller = raw.timestamp_us.is_some();
        let pending = PendingLc3 {
            ase_id: raw.ase_id,
            sequence: raw.sequence_number,
            timestamp_us: raw.timestamp_us.unwrap_or(receipt_timestamp_us),
            receipt_timestamp_us,
            timestamp_from_controller,
            frame: raw.frame,
        };
        received_frames = received_frames.wrapping_add(1);
        let Some((left, right)) = pairer.push(channel, pending) else {
            continue;
        };
        let left_after_right = left.timestamp_us.wrapping_sub(right.timestamp_us);
        let timestamp_us = if left_after_right < 0x8000_0000 {
            left.timestamp_us
        } else {
            right.timestamp_us
        };
        let gap_us = timestamp_us.wrapping_sub(last_timestamp_us);
        let sequence = left.sequence;
        let sequence_discontinuity = last_sequence.is_some_and(|last| sequence != last.wrapping_add(1));
        let mut flags = if !started || gap_us > 100_000 {
            FrameFlags::STREAM_START
        } else if sequence_discontinuity || gap_us > 15_000 {
            FrameFlags::DISCONTINUITY
        } else {
            FrameFlags::empty()
        };
        if queue_overflowed {
            flags = flags.union(FrameFlags::DISCONTINUITY);
        }

        let pair = StereoLc3Pair {
            left_ase_id: left.ase_id,
            right_ase_id: right.ase_id,
            sequence,
            timestamp_us,
            flags,
            left: left.frame,
            right: right.frame,
        };
        if LC3_TX_QUEUE.try_send(pair).is_err() {
            dropped_pairs = dropped_pairs.wrapping_add(1);
            queue_overflowed = true;
        } else {
            started = true;
            queue_overflowed = false;
        }

        last_sequence = Some(sequence);
        last_timestamp_us = timestamp_us;
        stereo_pairs = stereo_pairs.wrapping_add(1);
        if stereo_pairs % 40 == 0 {
            led.toggle();
        }
        if stereo_pairs % 200 == 0 {
            let now = Instant::now();
            let report_us = (now - report_started).as_micros();
            report_started = now;
            defmt::info!(
                "forwarded stereo_pairs={} received_lc3={} unpaired_lc3={} dropped_pairs={} hci_sequence={} last_200_pairs_us={}",
                stereo_pairs,
                received_frames,
                pairer.discarded,
                dropped_pairs,
                sequence,
                report_us
            );
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static HEAP_MEMORY: ConstStaticCell<[MaybeUninit<u8>; HEAP_SIZE]> =
        ConstStaticCell::new([const { MaybeUninit::uninit() }; HEAP_SIZE]);
    let heap_memory = HEAP_MEMORY.take();
    // SAFETY: `ConstStaticCell::take` grants this one call exclusive, permanent access to the
    // statically allocated heap region, and no other code accesses it directly afterward.
    unsafe { HEAP.init(heap_memory.as_ptr() as usize, HEAP_SIZE) }

    let mut config: config::Config = Default::default();
    config.clock_speed = config::ClockSpeed::CK128;
    config.hfclk_source = config::HfclkSource::ExternalXtal;
    config.lfclk_source = config::LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    static LED: StaticCell<Output> = StaticCell::new();
    let led = LED.init(Output::new(p.P0_02, Level::Low, OutputDrive::Standard));

    let mpsl_peripherals = mpsl::Peripherals::new(
        p.GRTC_CH7,
        p.GRTC_CH8,
        p.GRTC_CH9,
        p.GRTC_CH10,
        p.GRTC_CH11,
        p.TIMER10,
        p.TIMER20,
        p.TEMP,
        p.PPI10_CH0,
        p.PPI20_CH1,
        p.PPIB11_CH0,
        p.PPIB21_CH0,
    );
    let lfclk_config = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 50,
        skip_wait_lfclk_started: false,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_peripherals,
        Irqs,
        lfclk_config
    )));
    spawner.spawn(unwrap!(mpsl_task(&*mpsl)));

    let sdc_peripherals = sdc::Peripherals::new(
        p.PPI00_CH1,
        p.PPI00_CH3,
        p.PPI10_CH1,
        p.PPI10_CH2,
        p.PPI10_CH3,
        p.PPI10_CH4,
        p.PPI10_CH5,
        p.PPI10_CH6,
        p.PPI10_CH7,
        p.PPI10_CH8,
        p.PPI10_CH9,
        p.PPI10_CH10,
        p.PPI10_CH11,
        p.PPIB00_CH1,
        p.PPIB00_CH2,
        p.PPIB00_CH3,
        p.PPIB10_CH1,
        p.PPIB10_CH2,
        p.PPIB10_CH3,
    );
    let mut rng = cracen::Cracen::new_blocking(p.CRACEN);
    static SDC_MEMORY: StaticCell<sdc::Mem<16_384>> = StaticCell::new();
    let sdc_memory = SDC_MEMORY.init_with(sdc::Mem::new);
    let sdc = unwrap!(build_sdc(sdc_peripherals, &mut rng, mpsl, sdc_memory));

    static CIS_MANAGER: StaticCell<CisManager<NoopRawMutex, { basic_audio_sink::MAX_ASES }>> = StaticCell::new();
    let cis_manager = CIS_MANAGER.init(CisManager::new_passthrough());

    let flash = RefCell::new(Nvmc::new(p.RRAMC));
    let bond_store = rram_bond_store(&flash);

    let mut uart_config = uarte::Config::default();
    uart_config.baudrate = uarte::Baudrate::Baud1m;
    let uart = UarteTx::new(p.SERIAL21, Irqs, p.P1_14, uart_config);
    defmt::info!("stereo LC3 UART ready: SERIAL21 TX=P1.14 baud=1000000");
    interrupt::SWI01.set_priority(Priority::P6);
    let uart_spawner = UART_EXECUTOR.start(interrupt::SWI01);
    uart_spawner.spawn(unwrap!(uart_tx_task(uart)));

    let _ = select(
        forward_lc3(led, cis_manager),
        audio_sink::run(sdc, cis_manager, &bond_store),
    )
    .await;
}
