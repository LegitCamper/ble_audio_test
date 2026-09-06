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
