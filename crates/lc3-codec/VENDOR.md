# Local LC3 decoder optimizations

This directory vendors David Haig's `lc3-codec` 0.2.0, distributed under Apache-2.0.
The upstream [LICENSE](LICENSE), README, and original manifest are retained.

- Source: https://crates.io/crates/lc3-codec/0.2.0
- Upstream repository: https://github.com/ninjasource/lc3-codec
- Packaged commit: `65bfd7752d151a12d1e399e0edf4b84be136f083`
- Crate archive SHA-256: `ba01b209b6df33864b21e0d987689ad8e497b9a49f59bf75be0c1cf3bb47f2f4`

The RP2350 firmware depends on this directory with `default-features = false`.
The root workspace includes it for host tests; each consuming workspace retains
its own dependency lockfile and build profile. Cargo cache markers, the unused
package lockfile, and upstream editor settings are omitted.

Local changes:

- `decoder/arithmetic_codec.rs` and `decoder/side_info_reader.rs` use integer
  leading-zero counts instead of floating-point logarithms for bit counts.
- `decoder/long_term_post_filter.rs` uses exact rational integer arithmetic for
  pitch scaling and 7.5 ms gain selection, preserving the original rounding.
- `decoder/temporal_noise_shaping.rs` uses a 17-entry reflection coefficient table,
  preserving the upstream zero-index sentinel.
- `decoder/global_gain.rs` uses a 391-entry table for every possible gain exponent
  numerator (-245 through 145) from the five sampling indices and 8-bit gain index.
  Both tables together occupy 1,632 bytes of read-only data.
- `decoder/modified_dct.rs` computes normalization once per decoder construction.
- `decoder/fast_math_tables.rs`, its module declaration, and
  `examples/generate_decoder_tables.rs` provide reproducible tables. Entries store
  exact `f32` bits from the original expressions evaluated through
  `num-traits` 0.2.19 / `libm` 0.2.16, as pinned by both consuming lockfiles.
- Regression tests check the original math across all reflection/gain entries,
  frame-size bit widths, all 512 pitch indices and six sample rates at both frame
  durations, gain-selection thresholds, and arithmetic-range bit boundaries.
  Upstream codec and PCM test vectors are retained.
- The manifest disables publishing, removes the ignored package profile, and gates
  allocation-dependent examples. The encoder residual-bit return type spells out
  its existing lifetime to satisfy current Rust linting.

Regenerate the tables from the repository root using a temporary output file
(redirecting directly into the module would truncate it before Cargo builds):

```sh
cargo run -p lc3-codec --example generate_decoder_tables --locked > /tmp/ble-audio-decoder-tables.rs
cp /tmp/ble-audio-decoder-tables.rs crates/lc3-codec/src/decoder/fast_math_tables.rs
```

Validate from the repository root:

```sh
cargo test --workspace --locked
cargo test -p lc3-codec --no-default-features --locked
```

Build and lint each firmware from its own directory so Cargo loads the board's
target configuration:

```sh
cargo build --release --locked
cargo clippy --release --locked -- -D warnings
```

These checks establish build compatibility and numerical regression coverage.
RP2350 worst-case decode time, stereo stability, and XIP contention still require
hardware measurement; no timing improvement is claimed from host tests.
