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
