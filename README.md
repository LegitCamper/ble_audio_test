# nRF54L15 BLE Audio to USB DAC bridge

This repository contains a two-MCU stereo LE Audio path:

```text
LE Audio source
  -> two 48 kHz / 10 ms mono CIS/ASE streams (front left + front right)
  -> nRF54L15 (timestamp-pair left/right without decoding)
  -> two channel-tagged LC3 frames with one shared pair sequence
  -> 1 Mbaud UART (COBS + metadata + CRC-16)
  -> XIAO RP2350 core 1 (two-channel LC3 decoder writing planar S16LE)
  -> zero-copy bounded PCM ring
  -> XIAO RP2350 core 0 (direct USB-DPRAM fill + Full-Speed UAC2 host)
  -> TTGK TE-C USB DAC (3302:43e8), 48 kHz S16LE stereo
```

The USB host, DAC configuration, serial link, and earlier mono audio path have been exercised on
hardware. The RP2350 stereo decoder path builds cleanly but still needs end-to-end hardware timing
validation.

## Repository layout

- `crates/audio-link`: allocator-free, corruption-resynchronizing UART protocol shared by both MCUs
- `firmware/nrf54l15`: Trouble Audio unicast sink and selected-channel LC3 transmitter
- `firmware/xiao-rp2350`: dual-core LC3 decoder/USB bridge and TE-C-specific UAC2 host
- `docs/serial-protocol.md`: exact version-1 wire format

The firmware directories are separate Cargo workspaces. Trouble Audio uses a newer upstream Embassy
revision while the RP2350 USB-host implementation uses the pinned LegitCamper host fork; combining
them would select incompatible global time drivers.

## What is implemented

The nRF firmware advertises the known-working pair of 48 kHz sink ASEs, receives both LC3 streams,
and retains short per-channel FIFOs while pairing left/right frames received within 5 ms. This
preserves ordering when the controller delivers several frames from one CIS in a burst. A dedicated
UART executor sends each pair together and overlaps DMA output with ISO reception. The advertised
PAC record allows up to 155 octets per codec frame; two maximum frames every 10 ms consume less
than 36% of a 1 Mbaud 8-N-1 link after framing.

The RP2350 dedicates core 1 to UART parsing and a two-channel native Rust `lc3-codec` decoder.
Matching left/right frames decode directly into planar channels in the final PCM block. The block
is published through Embassy's fixed 16-slot zero-copy SPSC ring, so no 1,920-byte PCM block is moved
between cores. Core 0 owns USB and reads each ring slot in place after a 60 ms prebuffer. It
interleaves both planes while filling the controller's dedicated USB DPRAM, eliminating intermediate
PCM and USB packet buffers; the final roughly 192-byte SRAM-to-DPRAM write is required by RP2350
hardware.

The RP2350 currently uses an experimental 300 MHz target with a 1.25 V core setting. This is twice
the chip's rated 150 MHz operating frequency. It may improve decoder headroom, but temperature,
flash execution, USB operation, and long-duration stability must be validated on each board.

The firmware uses the no-allocator configuration of the native Rust `lc3-codec` 0.2 decoder. Its
two-channel working memory is statically allocated, and each decoder channel writes directly into its
plane in the next PCM ring slot. No C compiler, FFI binding, C runtime, or codec heap allocation is
part of the RP2350 build.

The RP2350 image currently runs on the dual Cortex-M33 cores, not Hazard3. The LC3 implementation
uses single-precision floating point throughout; RP2350's M33 cores have a hardware single-precision
FPU, while Hazard3 does not. The pinned Embassy RP USB-host HAL is also Cortex-M-only. Moving this
specific pipeline to RISC-V now would require a different USB-host stack and would make LC3's `f32`
work software-emulated, so M33 is the useful performance target for LC3 decoding.

The USB class remains intentionally narrow. It accepts only VID:PID `3302:43e8`, UAC2 playback
interface 2 alternate 1, stereo PCM, 16-bit subslots, and Full Speed. It selects the 48 kHz clock on
entity 1, sends PCM to the asynchronous isochronous OUT endpoint, and follows the four-byte 16.16
explicit feedback endpoint. Because UAC2 advertises supported rates through the clock entity rather
than the streaming descriptors, the host queries the Sampling Frequency Control's `RANGE`, then
reads `CUR` back after setting it, and refuses the device if 48 kHz is missing or does not stick. A
device that stalls `RANGE` outright is still tried. The Embassy host fork rejects isochronous
channel allocation at runtime, so `iso.rs` supplies a direct RP2 EPX transaction path paced from
USB SOF.

## Wiring

| Signal | nRF54L15 | XIAO RP2350 |
|---|---|---|
| Stereo LC3 data | `P1.14` / `SERIAL21 TX` | `D7` / `GPIO1` / `UART0 RX` |
| Reference | GND | GND |

UART is 1,000,000 baud, 8-N-1, transmit-only. The nRF54L15 DK defaults its `VDD:IO` GPIO domain to
1.8 V, below the XIAO RP2350's guaranteed input-high threshold. Use the nRF Connect Board
Configurator to set `nRF:VDD`/`VDD:IO` to 3.3 V before connecting the UART wire. Only TX and a common
ground are required; do not join the boards' 3.3 V supply pins while both boards are independently
USB-powered. There is no hardware flow control in protocol version 1.

The XIAO's native USB data pins are used for host mode. A normal USB-C cable is insufficient: the
board needs a host/OTG fixture that presents the proper Type-C source role, supplies current-limited
5 V VBUS to the DAC, and prevents back-powering either board. Do not simultaneously use the native
USB connector for flashing or serial while it is wired as the DAC host; use SWD and RTT for bring-up.

## Build and test

The pinned toolchain and targets are declared in `rust-toolchain.toml`.

```sh
# Shared protocol tests and lint
cargo test --locked
cargo clippy --all-targets --locked -- -D warnings

# nRF54L15 application-core image
cd firmware/nrf54l15
cargo build --release --locked
cargo clippy --release --locked -- -D warnings

# XIAO RP2350 Cortex-M33 image
cd ../xiao-rp2350
cargo build --release --locked
cargo clippy --release --locked -- -D warnings

# With picotool 2.x installed and the XIAO mounted in BOOTSEL mode,
# package an RP2350 UF2, copy it to the board, and reboot it
cargo run --release --locked

# USB-only 1 kHz diagnostic image (bypasses UART and LC3)
cargo build --release --locked --features usb-test-tone
```

The resulting ELF files are:

- `firmware/nrf54l15/target/thumbv8m.main-none-eabihf/release/ble-audio-nrf54l15`
- `firmware/xiao-rp2350/target/thumbv8m.main-none-eabihf/release/ble-audio-xiao-rp2350`

Flash them with the probe/bootloader flow appropriate to the exact boards. RTT is the detailed
diagnostic output. The XIAO RP2350's active-low yellow user LED on GPIO25 turns on once stereo PCM
is reaching successful USB audio writes.

## Required hardware gates

1. **Check the DAC at Full Speed.** Connect the DAC through a USB 1.1 Full-Speed hub (or otherwise
   force Full Speed) and capture `lsusb -v`; interface 2 alt 1 must expose stereo 16-bit PCM, an
   isochronous OUT endpoint, and a four-byte feedback IN endpoint. Also confirm the clock source
   on entity 1 accepts 48 kHz: `lsusb -v` does not list UAC2 rates, so read them from the device
   with a Sampling Frequency Control `RANGE` request, or watch the RTT `DAC clock subrange` lines
   this firmware logs at configure time.
2. **Validate the host fixture.** Confirm 5 V VBUS, Type-C role resistors, current limiting, and no
   backfeed before attaching the XIAO and DAC together.
3. **Measure RP2350 decode timing and cadence.** RTT reports `max_stereo_decode_us`,
   `last_100_blocks_us`, `last_1000_packets_us`, `sent_pcm_frames`, `buffered_blocks`,
   `failed_lc3`, `dropped_pcm`, `unpaired_lc3`, and `rejected_wire`. One hundred decoded blocks
   and 1,000 USB packets should each take approximately one second. The paired left/right decode
   must stay comfortably below the 10,000 us codec interval; a ring can absorb jitter but cannot
   fix average decode throughput slower than real time.
4. **Watch the nRF link.** A sustained `dropped_lc3` count means ISO reception plus UART transfer is
   not keeping up.
5. **Validate EPX isochronous traffic.** Watch USB `underflows` and transaction errors during the
   first physical-device run.

## Current constraints

- The source must establish two 48 kHz, 10 ms mono ASEs allocated front left and front right.
- Left/right frames separated by more than 5 ms at the nRF forwarding task are not paired.
- DAC playback is fixed to stereo S16LE at 48 kHz; 24/32-bit alternates and microphone capture are
  ignored.
- There is no UART return channel, flow control, retransmission, or sample-rate correction between
  MCUs. CRC/COBS detects damage and restores frame boundaries; USB underflow produces silence.
- Disconnect/reconnect and malformed traffic paths are handled, but the stereo decoder and
  direct isochronous engine still require physical-device testing together.
