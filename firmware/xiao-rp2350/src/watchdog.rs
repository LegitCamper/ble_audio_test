//! Recovery requires progress from both application tasks, including their normal idle waits.

use core::future::Future;
use core::pin::pin;

use embassy_futures::select::{Either, select};
use embassy_rp::pac;
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_time::{Duration, Timer};
use portable_atomic::{AtomicU8, Ordering};

const ALL_TASKS: u8 = 3;
const SCRATCH_MAGIC: u32 = 0x4155_4457;
static PROGRESS: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy)]
pub enum Task {
    Usb = 1,
    Audio = 2,
}

pub fn progress(task: Task) {
    PROGRESS.fetch_or(task as u8, Ordering::Relaxed);
}

/// Only use for waits where an absent peer is healthy, never for USB control transfers.
/// Keep the original future alive across timer ticks so UART/event waits aren't cancelled.
pub async fn idle_wait<F: Future>(task: Task, future: F) -> F::Output {
    let mut future = pin!(future);
    loop {
        progress(task);
        match select(future.as_mut(), Timer::after_millis(250)).await {
            Either::First(result) => return result,
            Either::Second(()) => {}
        }
    }
}

/// Scratch 0..2 survive watchdog resets; 4..7 are reserved for ROM reboot conventions.
pub fn start(watchdog: &mut Watchdog) -> bool {
    let timed_out = watchdog.reset_reason() == Some(ResetReason::TimedOut);
    let previous_count = if watchdog.get_scratch(0) == SCRATCH_MAGIC {
        watchdog.get_scratch(1)
    } else {
        0
    };
    let missing_tasks = if timed_out { watchdog.get_scratch(2) } else { 0 };
    let reset_count = previous_count.saturating_add(u32::from(timed_out));
    watchdog.set_scratch(0, SCRATCH_MAGIC);
    watchdog.set_scratch(1, reset_count);
    watchdog.set_scratch(2, 0);
    watchdog.pause_on_debug(true);
    watchdog.start(Duration::from_secs(5));

    // The pinned Embassy driver uses the RP2040 PSM mask even on RP2350: it omits
    // PROC0/PROC1 and resets the oscillators. Match pico-sdk's RP2350 watchdog mask.
    pac::PSM.wdsel().write(|w| {
        w.0 = 0x01ff_ffff;
        w.set_rosc(false);
        w.set_xosc(false);
    });
    defmt::info!(
        "watchdog: timeout_reset={} count={} missing_tasks={} (USB=1 audio=2)",
        timed_out,
        reset_count,
        missing_tasks
    );
    timed_out
}

#[embassy_executor::task]
pub async fn run(mut watchdog: Watchdog) -> ! {
    loop {
        Timer::after_millis(250).await;
        let seen = PROGRESS.load(Ordering::Relaxed);
        watchdog.set_scratch(2, u32::from(ALL_TASKS & !seen));
        if seen == ALL_TASKS {
            // Require new progress from each task for every hardware reload.
            PROGRESS.fetch_and(!ALL_TASKS, Ordering::Relaxed);
            watchdog.feed();
        }
    }
}
