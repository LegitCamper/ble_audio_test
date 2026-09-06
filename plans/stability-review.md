# Firmware stability review

Reviewed 2026-09-05 against `f3eec13` plus this change. Scope: both firmware entry
points, UART/LC3 transport and decoder ingress, USB playback/control paths, volume
recovery, and the pinned HAL watchdog/USB implementations. This is a code review
and host/build validation, not a hardware soak result. RF/controller internals,
electrical behavior, full codec conformance, and stack high-water usage were not
exhaustively audited. The cause of the reported XIAO deaths remains unconfirmed.

## Addressed findings

| Finding | Evidence | Impact / response | Confidence |
| --- | --- | --- | --- |
| USB controls can wait forever | Pinned Embassy `760eb3b`, `embassy-rp/src/usb/host.rs:261,285,691`; local `usb_audio.rs` control operations | USB task stops contributing watchdog progress during a stuck setup/control/SOF operation. Recover by full MCU reset; do not cancel a control future and reuse uncleared hardware state. | High |
| HAL watchdog uses RP2040 reset mask on RP2350 | Pinned `embassy-rp/src/watchdog.rs`, `configure_wdog_reset_triggers`; RP2350 PSM PROC0/PROC1 are bits 23/24 | Override the mask immediately after starting the watchdog to reset all RP2350 domains except ROSC/XOSC, matching pico-sdk. Without this, calling the HAL watchdog API alone does not select the intended system reset. | High |
| Truncated LC3 can panic | `crates/lc3-codec/src/decoder/buffer_reader.rs`, `read_tail_bool`; `lc3_decoder.rs`, truncated-side-info test | Reproduced a four-byte frame indexing past the buffer in release mode. Checked subtraction now returns the existing error so the decoder can conceal it. Preserve arithmetic head read-ahead; original PCM vectors pass. | High, reproduced |
| Volume/mute lost on XIAO reset or DAC reconnect | `firmware/xiao-rp2350/src/audio.rs`, volume mailbox; both firmware `main.rs` volume paths | Retain latest volume, replay from nRF every second, deduplicate USB requests, and output silence until applied. Failed controls retry at most once per second. Update both images together. | High |
| Startup can miss a connected DAC | Pinned USB driver `wait_for_device_event` observes a change from its initial state | Poll actual port attachment for startup/reconnect, including a DAC already connected throughout watchdog reset and LED flashes. | High |
| Debugger can leave RTT blocking | `defmt-rtt` crate blocking-mode documentation | Enable `disable-blocking-mode` in both manifests. An absent probe alone is not a cause: RTT starts nonblocking. | High |

## Remaining priorities

| Priority | Finding / evidence | Impact | Effort | Fix risk | Confidence |
| --- | --- | --- | --- | --- | --- |
| 1 | Experimental 300 MHz / 1.25 V in XIAO `main.rs` | Clock/voltage margin is a plausible source of hard faults or freezes. Compare with the `stock-clock` 150 MHz / 1.10 V build using the same wiring, DAC and source. Existing code records underflows at 225 MHz; rated-clock playback may be intermittent. | S to compare, L to recover sufficient rated-clock decoder throughput | Low for comparison; high for decoder restructuring | High that it is overclocked; unconfirmed crash cause |
| 2 | No on-board reset/soak validation | Compilation cannot establish successful watchdog reboot, USB re-enumeration, BLE reconnection, or absence of false timeouts under load. Run the checks below before relying on recovery. | M | Low | High |
| 3 | nRF `watchdog::idle_wait` around `receive_lc3` cannot distinguish idle from an async BLE-stack wedge | Synchronous executor stalls and stuck UART writes reset, but a BLE future stuck pending can coexist with healthy forwarding idle ticks. Add state-specific controller/GATT operation deadlines if reproduced. Do not use absence of audio alone as a fault. | M | Medium; false resets during normal idle must be avoided | High coverage limit |
| 4 | XIAO `main.rs` continues after isochronous errors while device reports connected | Endless promptly returning write errors still count as task progress. Consider a consecutive-error threshold with reset/re-enumeration after measuring transient failures on the actual DAC. | S | Medium; avoid resetting for tolerated transient errors | High coverage limit |
| 5 | LC3 `DecoderChannel::decode` maps parse errors to PLC and returns `Ok` | Firmware `failed_lc3` excludes corruption concealed internally. Add separate concealment/corruption diagnostics if transport or codec failures remain suspected. | S | Low | High |

## Watchdog behavior and limits

- Timeout is five seconds. RP reload polling is every 250 ms and requires a new
  contribution from both USB and audio tasks. A contribution already pending when
  a task fails may permit one final reload. Expect reset within roughly 5.5 seconds
  of a stall, plus boot/setup time; this is not a hard real-time measured bound.
- nRF WDT1 has two hardware handles, owned by forwarding and UART transmission.
  Both must reload. Idle futures remain pinned across 250 ms timer ticks, avoiding
  cancellation of UART/event waits. UART writes do not reload while pending.
- Both watchdogs start after HAL/clock initialization. They cannot cover a hang
  before that point. They run during normal executor sleep and pause during debug
  halt; deliberate debugger halts are unsuitable for proving watchdog expiry.
- The XIAO reports one startup LED flash normally and three following a watchdog
  timeout. Scratch 0..2 store a signature, timeout count, and last observed missing
  task mask (USB=1, audio=2); 4..7 remain available for ROM reboot conventions.
  A stopped supervisor cannot update the mask, so it is a clue, not a crash trace.
  Scratch is not durable across power removal. nRF logs raw reset reason bits and
  clears them at startup.
- A watchdog does not prove the source of a reset or correct bad power/clock margin.
  Persistent failures can produce repeated resets. BOOTSEL remains the XIAO
  recovery/flashing route without a probe.

## Verification

From the repository root:

```sh
cargo test --workspace --locked
cargo test -p lc3-codec --no-default-features --locked
```

From each firmware directory:

```sh
cargo build --release --locked
cargo clippy --release --locked -- -D warnings
cargo fmt --check
```

Additionally, from `firmware/xiao-rp2350`:

```sh
cargo build --release --locked --features stock-clock
cargo clippy --release --locked --all-features -- -D warnings
```

Host results: 99 workspace tests, including the three new LC3 regressions; 45 codec
tests also pass without allocation. Both firmware release builds and firmware
Clippy checks pass. These tests do not execute MCU watchdogs or model USB hardware.
Firmware formatting and the edited codec files pass formatting checks. The broader
`cargo fmt --all --check` reports pre-existing layout differences in vendored
`decoder/noise_filling.rs` and `encoder/bitstream_encoding.rs`; these were left alone.

## Hardware acceptance checks (not yet run)

1. Flash both images. Boot with the DAC already plugged in. Verify one XIAO startup
   flash, DAC enumeration, and volume-correct audio after BLE connects.
2. Leave each board powered with no audio source and with the DAC absent for at
   least ten minutes. Neither should reset merely because an external peer is idle.
3. Exercise an XIAO-only reset while the phone stays connected to the nRF, first
   muted and then at a low volume. Verify silence until volume restores, with no
   full-gain burst. Repeat unplug/replug of the DAC.
4. In disposable diagnostic builds, inject a permanent loop in each XIAO task
   separately after startup. Verify automatic reboot and three LED flashes. Also
   inject an async forever-pending wait in the USB streaming/control path; the
   supervisor must not mistake a responsive executor for healthy USB progress.
5. Likewise stop nRF forwarding and UART transmission separately, including a
   never-completing UART write. Verify WDT1 reset and subsequent BLE reconnect.
   Use source-level fault injection, not a debugger halt, because debug pause is
   intentional. Remove all injected faults before the playback build.
6. Unplug the DAC during repeated volume changes/setup. Verify return to waiting
   or watchdog recovery, then successful re-enumeration after replug.
7. Run several hours of stereo playback with the normal and stock-clock images
   under the same setup. Record reset flashes, uptime, rebuffering, decode maxima,
   and UART/PCM losses where telemetry is available. Separate audio starvation
   from MCU reset/freeze when comparing the two clocks.

## Considered and rejected

- "No probe means RTT will block" is not supported: its initial mode is nonblocking.
- No evidence ties the reproduced malformed short frame to normal negotiated-length
  traffic or to the user's observed failures.
- Feeding from a standalone timer/interrupt alone is insufficient: stuck application
  futures may still allow that timer to run. Missing incoming audio alone is also
  insufficient evidence of failure.
