# Trouble Audio local patch

This directory is copied from
[`LegitCamper/trouble_audio`](https://github.com/LegitCamper/trouble_audio) commit
`894a43024f40f150469c83d5df58d81781044a3c` and remains under the included Apache-2.0 license.

The local patch is intentionally limited to the CIS passthrough boundary:

- preserve the HCI ISO SDU sequence number and optional timestamp in `RawLc3`;
- reject fragmented, status-bad, and length-mismatched SDUs instead of decoding partial data;
- make the copied crate's `trouble-host` dependency explicit outside its original workspace.

Remove the `[patch]` entry in `firmware/nrf54l15/Cargo.toml` when upstream exposes equivalent
metadata and validation.
