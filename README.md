# Trouble Audio BLE-to-USB example

This is the in-depth hardware example for
[`trouble_audio`](https://github.com/LegitCamper/trouble_audio). It receives stereo LE Audio on an
nRF54L15, sends the LC3 frames to an RP2350, decodes them, and plays them through a USB DAC.

## Hardware

- Nordic nRF54L15 DK
- Seeed Studio XIAO RP2350
- Full-Speed, class-compliant UAC1 or UAC2 USB DAC supporting 48 kHz, 16-bit stereo
- USB-C host/OTG fixture with safe 5 V VBUS for the DAC
- One UART wire plus common ground

Connect nRF54L15 `P1.14` / `SERIAL21 TX` to XIAO `D7` / `GPIO1` / `UART0 RX`. UART runs at
1,000,000 baud, 8-N-1. Set the nRF54L15 DK `VDD:IO` to 3.3 V first. Do not connect the boards'
3.3 V pins when both are USB-powered.

## What works

- Two 48 kHz, 10 ms mono LE Audio streams are received and paired as stereo.
- Paired LC3 frames cross the UART with COBS framing, metadata, and CRC-16.
- The XIAO decodes stereo LC3 and streams 48 kHz S16LE audio to compatible USB DACs.
- UAC1/UAC2 descriptor discovery, explicit feedback, volume control, bonding, and reconnect paths
  are implemented.
- Shared protocol and USB descriptor tests pass; both firmware images build.

USB hosting, DAC setup, UART transport, and the earlier mono path have run on hardware. Full stereo
end-to-end timing and long-duration stability still need hardware validation; the RP2350 currently
runs at an experimental 300 MHz overclock.

## Watchdog recovery and clock diagnosis

Both firmwares start a five-second hardware watchdog immediately after HAL initialization.
The XIAO requires fresh progress from both USB and UART/LC3 tasks before reloading it;
the nRF requires independent reloads from the main forwarding path and UART transmitter.
Normal waits for a DAC, BLE audio, or UART input remain healthy indefinitely. USB setup,
USB control transfers, SOF waits, decoding, and UART writes cannot feed through a hang.
Watchdogs run during normal sleep and pause when a debugger halts a core. RTT logging
is forced nonblocking so a disconnected logging session cannot freeze the firmware.

The XIAO yellow LED flashes **once on normal startup, three times after a watchdog
timeout**, then returns to its playback indication. Watchdog scratch registers retain
a timeout count and the last missing-task mask for RTT diagnostics; these are not a
power-loss-persistent crash log. The nRF logs and clears its hardware reset-reason bits.
After an XIAO reset or DAC reconnect, packets remain silent until volume is applied.
The nRF replays current volume/mute every second; unchanged settings do not cause
repeated USB control requests. Update both boards together for this recovery behavior.

Default XIAO builds retain **300 MHz / 1.25 V** because the existing implementation
records stereo underflows at 225 MHz. For comparison at rated **150 MHz / 1.10 V**,
build from `firmware/xiao-rp2350`:

```sh
cargo build --release --locked --features stock-clock
```

Use the usual BOOTSEL UF2 flashing procedure, or `cargo run --release --locked
--features stock-clock` with the BOOTSEL drive mounted. Omit `--features stock-clock`
to restore the playback build. At 150 MHz expect possible audio underflows: compare
resets/freezes separately from uninterrupted playback. A stable diagnostic run would
implicate the clock/voltage/load combination, but would not isolate its exact cause.

See the [stability review and hardware checks](plans/stability-review.md) for findings,
watchdog coverage limits, and the remaining validation work.

## Audio timing and local codec

The startup/rebuffer threshold is six 10 ms PCM blocks (60 ms). USB output consumes
PCM in chunks per packet, and ring occupancy controls interpolated sample insertion
or removal. Sequence gaps trigger LC3 packet-loss concealment for up to four missing
stereo blocks per gap. The nRF queues raw LC3 pairs and frames them in its UART task.

The RP2350 uses a locally vendored Apache-2.0 LC3 decoder with integer bit/pitch
math, fixed TNS/global-gain tables, and cached MDCT normalization. See
[vendor provenance and validation commands](crates/lc3-codec/VENDOR.md).

Use the firmware counters to guide further optimization:

- `max_stereo_decode_us > 10_000` indicates a stereo decode exceeded one frame period.
- Sustained `last_100_blocks_us > 1_000_000` indicates that the producer is delivering
  fewer than 100 stereo blocks per second; correlate with transport-loss counters.
- Falling `buffered_blocks` while `inserted_frames` rises indicates correction cannot
  keep up with the supply deficit; correlate with decode timing and sequence gaps.
- `dropped_pairs`, `unpaired_lc3`, and `rejected_wire` identify transport/pairing losses.
  `concealed_blocks` and `max_stereo_plc_us` show concealment activity and its cost.

SRAM placement, splitting channel decoding across cores, and further nRF tuning
remain options if hardware measurements show insufficient headroom.
