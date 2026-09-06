//! Separate hardware reload handles supervise the main and UART application paths.

use core::future::Future;
use core::pin::pin;

use embassy_futures::select::{Either, select};
use embassy_nrf::wdt::WatchdogHandle;
use embassy_time::Timer;

/// Idle BLE/queue waits are healthy. Preserve the future across timer ticks.
/// Do not use for UART writes: a wedged transfer must let its handle expire.
pub async fn idle_wait<F: Future>(handle: &mut WatchdogHandle, future: F) -> F::Output {
    let mut future = pin!(future);
    loop {
        handle.pet();
        match select(future.as_mut(), Timer::after_millis(250)).await {
            Either::First(result) => return result,
            Either::Second(()) => {}
        }
    }
}
