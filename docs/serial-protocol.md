# Audio serial protocol version 1

Each message is a raw body followed by CRC-16/CCITT-FALSE, COBS encoded, and terminated by `0x00`.
All multi-byte integers are little-endian. The largest payload is 720 bytes and the decoder uses
fixed storage, so corrupt input cannot grow memory use.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | version (`1`) |
| 1 | 1 | encoding (`1` = LC3, `2` = packed signed 12-bit little-endian mono PCM) |
| 2 | 1 | flags: bit 0 discontinuity, bit 1 stream start |
| 3 | 1 | channel: `0` left, `1` right, `2` mono, `255` unknown |
| 4 | 1 | ASE identifier |
| 5 | 1 | frame duration: `0` = 7.5 ms, `1` = 10 ms |
| 6 | 2 | wrapping sequence number |
| 8 | 4 | wrapping nRF receive timestamp in microseconds |
| 12 | 4 | sample rate in hertz |
| 16 | 2 | payload length |
| 18 | N | audio payload |
| 18 + N | 2 | CRC over header and payload |

Encoding 1 accepts at most 155 bytes. The nRF sends a left message followed by a right message for
each stereo pair. Both messages retain the controller-provided HCI ISO sequence. They use the HCI
timestamp when present and otherwise the nRF receive timestamp. The RP2350 publishes PCM only after
matching the two values by sequence. A missing or corrupt message therefore drops a pair instead of
shifting channel alignment.

Encoding 2 remains supported for compatibility with the earlier mono path. It contains exactly 480
samples in 720 bytes: each three-byte group stores two signed 12-bit two's-complement samples, with
the first sample in bits 0..11 and the second in bits 12..23.

The receiver drops malformed/version-mismatched/CRC-failed messages. If bytes accumulate beyond one
maximum frame, it ignores input through the next zero delimiter and then resumes normally.
