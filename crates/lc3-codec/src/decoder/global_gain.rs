// Local modification: replace per-frame powf with the equivalent fixed-domain table.
use super::fast_math_tables::GLOBAL_GAIN;
use crate::common::complex::Scaler;

// checked against spec

/// Adjusts the loudness for all spectral lines in the frame
///
/// # Arguments
///
/// * `frame_num_bits` - Number of bits in the frame
/// * `fs_ind` - Sampling frequency index (e.g. 4 for 48khz)
/// * `global_gain_index` - Computed global gain index (e.g 204)
/// * `spec_lines` - All the spectral lines in a frame to be mutated
pub fn apply_global_gain(frame_num_bits: usize, fs_ind: usize, global_gain_index: usize, spec_lines: &mut [Scaler]) {
    let fs = fs_ind as i32 + 1;
    let nbits = frame_num_bits as i32;
    let gg_off = -((nbits / (10 * fs)).min(115)) - 105 - (5 * fs);
    // An 8-bit gain index and sampling indices 0..=4 give a numerator in -245..=145.
    let gg = GLOBAL_GAIN[(global_gain_index as i32 + gg_off + 245) as usize];

    for f in spec_lines.iter_mut() {
        *f *= gg;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn gain_table_should_match_original_math_for_all_frame_sizes_and_indices() {
        for (index, gain) in GLOBAL_GAIN.iter().enumerate() {
            let expected = num_traits::real::Real::powf(10.0_f32, (index as i32 - 245) as f32 / 28.0);
            assert_eq!(gain.to_bits(), expected.to_bits(), "table index={index}");
        }
        for fs_ind in 0..5 {
            for frame_bytes in 0..=400 {
                let fs = fs_ind as i32 + 1;
                let gg_off = -((frame_bytes as i32 * 8 / (10 * fs)).min(115)) - 105 - 5 * fs;
                for gain_index in 0..=255 {
                    let expected = num_traits::real::Real::powf(10.0_f32, (gain_index as f32 + gg_off as f32) / 28.0);
                    let mut actual = [1.0];
                    apply_global_gain(frame_bytes * 8, fs_ind, gain_index, &mut actual);
                    assert_eq!(
                        actual[0].to_bits(),
                        expected.to_bits(),
                        "fs_ind={fs_ind}, bytes={frame_bytes}, gain={gain_index}"
                    );
                }
            }
        }
    }

    #[test]
    fn global_gain_decode() {
        let mut spec_lines = [1.0, 10.0, 100.0];

        apply_global_gain(1200, 4, 204, &mut spec_lines);

        assert_eq!(spec_lines, [61.0540199, 610.540199, 6105.40199])
    }
}
