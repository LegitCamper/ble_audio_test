# nRF54L15 BLE Audio to USB DAC bridge

This repository contains a two-MCU stereo LE Audio path:

```text
LE Audio source
  -> two 48 kHz / 10 ms mono CIS/ASE streams (front left + front right)
  -> nRF54L15 (HCI-sequence-pair left/right without decoding)
  -> two channel-tagged LC3 frames carrying an aligned HCI sequence and optional timestamp
  -> 1 Mbaud UART (COBS + metadata + CRC-16)
  -> XIAO RP2350 core 1 (two-channel LC3 decoder writing planar S16LE)
  -> zero-copy bounded PCM ring
  -> XIAO RP2350 core 0 (direct USB-DPRAM fill + Full-Speed UAC1/UAC2 host)
  -> class-compliant USB DAC, 48 kHz S16LE stereo
```

The USB host, DAC configuration, serial link, and earlier mono audio path have been exercised on
hardware. The RP2350 stereo decoder path builds cleanly but still needs end-to-end hardware timing
validation.

## Repository layout

- `crates/audio-link`: allocator-free, corruption-resynchronizing UART protocol shared by both MCUs
- `crates/uac-descriptor`: allocation-free USB Audio descriptor parsing, playback-path selection,
  and feature-unit volume discovery
- `firmware/nrf54l15`: Trouble Audio unicast sink, Volume Control Service, and selected-channel
  LC3 transmitter
- `firmware/xiao-rp2350`: dual-core LC3 decoder and descriptor-driven USB Audio host
- `docs/serial-protocol.md`: exact version-1 wire format

The firmware directories are separate Cargo workspaces. The nRF firmware pins its `trouble_audio`
and `trouble` dependencies to upstream Git revisions. Trouble Audio uses a newer upstream Embassy
revision while the RP2350 USB-host implementation uses the pinned LegitCamper host fork; combining
them would select incompatible global time drivers.

## What is implemented

The nRF firmware advertises the known-working pair of 48 kHz sink ASEs, receives both LC3 streams,
and retains short per-channel FIFOs. Because each CIS has an independent HCI ISO sequence origin,
the first timestamp-matched pair establishes a fixed cross-CIS sequence offset. Subsequent frames
are paired by their aligned sequence. The HCI timestamp is retained when present; otherwise the
receive timestamp is used. Invalid, fragmented, and length-mismatched SDUs are rejected before
forwarding. This preserves ordering and resynchronizes after a loss even when the controller
delivers several frames from one CIS in a burst. A dedicated UART executor sends each pair together
and overlaps DMA output with ISO reception. The advertised PAC record allows up to 155 octets per
codec frame; two maximum frames every 10 ms consume less than 36% of a 1 Mbaud 8-N-1 link after
framing.

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

The USB host discovers any Full-Speed, class-compliant UAC1 or UAC2 alternate setting that exposes
48 kHz stereo signed 16-bit PCM. Interface numbers, alternate settings, endpoint addresses, UAC2
control interfaces, and clock-source entities all come from descriptors rather than device-specific
constants. UAC1 discrete and continuous sample-rate declarations are supported. Fixed, adaptive,
and synchronous endpoints run at the nominal 48 frames per USB frame; asynchronous endpoints must
provide a three-byte 10.14 or four-byte 16.16 explicit-feedback endpoint. Selection lives in
`crates/uac-descriptor` rather than in the firmware so it is exercised by host tests, including
truncated and length-corrupted descriptors that must be rejected instead of faulting the host. UAC2 clock controls are
read before modification, read-only 48 kHz clocks are accepted, and writable clocks are verified
after configuration. Volume originates on the nRF, which exposes the LE Audio Volume Control Service so the phone owns
the slider. Each Volume Control Point write is read back from the Volume State characteristic and
forwarded to the RP2350 as a kind-3 serial frame, so the value that crosses the link is the one the
profile state machine settled on, including its clamping of relative steps. Mute travels separately
from the level, because unmuting has to restore the previous level rather than zero.

The nRF persists the latest bond in RRAM. On boot and after a disconnect it first sends a
high-duty directed advertisement to that saved peer for one second, then a hard timer cancels it
and starts normal discoverable, scannable advertising so a different source can connect. If the
saved peer has used "Forget This Device", its next pairing request clears the stale runtime and
RRAM bond before saving the replacement. The peripheral cannot initiate a BLE link, but directed
advertising gives the saved phone a fast, targeted reconnect opportunity.

On the USB side, volume has exactly one owner, chosen once at configure time. If the
device exposes a feature unit fed by this stream that claims a writable volume control, the host
probes its range and, when that succeeds, hands volume to the DAC. A device that advertises no such
control, or advertises one whose probe stalls, leaves the RP2350 applying a Q15 gain to each sample
on the way into USB DPRAM instead. Both paths map nonzero VCS levels onto the same -30 dB to 0 dB
range, keeping the lower half of the phone slider audible; level zero and VCS mute are exact
silence. Because the choice is made before streaming starts and never changes, attenuation is
never applied twice. The Embassy host fork
rejects isochronous channel allocation at runtime, so `iso.rs` supplies a direct RP2 EPX transaction
path paced from USB SOF. A transient isochronous transaction failure drops only that packet and the
next SOF retries in place; a real detach exits playback and waits for the DAC to reconnect.

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
# Shared protocol and USB descriptor tests, plus lint
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

1. **Check each DAC at Full Speed.** Connect it through a USB 1.1 Full-Speed hub (or otherwise force
   Full Speed) and capture `lsusb -v`. It must expose a UAC1 or UAC2 playback alternate with stereo
   16-bit PCM at 48 kHz and an isochronous OUT packet capacity of at least 192 bytes. Asynchronous
   OUT endpoints must also expose a three- or four-byte explicit-feedback IN endpoint. Watch the RTT
   `USB Audio configured` line to confirm the selected UAC version, interface, alternate, and
   endpoint addresses.
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
6. **Confirm the volume owner and path.** The RTT `volume_owner=` field reports which side applies
   volume. Move the phone's volume slider and confirm a `sent volume` line on the nRF and a matching
   `volume received` line on the RP2350. A DAC that advertises a feature unit but fails the probe
   must fall back to `volume_owner=rp2350` rather than losing volume control entirely.

## Current constraints

- The source must establish two 48 kHz, 10 ms mono ASEs allocated front left and front right.
- Initial left/right alignment requires timestamps or local receipt times within 5 ms.
- DAC playback requires Full-Speed UAC1/UAC2 stereo S16LE at 48 kHz; 24/32-bit, High-Speed-only,
  implicit-feedback-only, vendor-specific, and microphone interfaces are ignored.
- Volume is not persisted across power cycles, so the sink reports its factory default on every
  boot and the Volume Flags characteristic stays clear.
- There is no UART return channel, flow control, retransmission, or sample-rate correction between
  MCUs. CRC/COBS detects damage and restores frame boundaries; USB underflow produces silence.
- Disconnect/reconnect and malformed traffic paths are handled, but the stereo decoder and
  direct isochronous engine still require physical-device testing together.
