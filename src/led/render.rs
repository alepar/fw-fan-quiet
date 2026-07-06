//! Pure rendering for the wattage waterfall. Time runs along the tall (34)
//! axis and wattage along the short (9) axis: each time row is a horizontal
//! bar whose lit width is the wattage fraction, and successive samples scroll
//! along the tall axis. Newest sample first. No I/O — unit-tested in isolation.

/// Matrix geometry. WIDTH is the short (wattage) axis addressed by `SendCol`'s
/// column index; HEIGHT is the tall (time) axis — the 34 LEDs within a column.
pub const WIDTH: usize = 9;
pub const HEIGHT: usize = 34;

/// Grid orientation relative to the panel's physical mounting, calibrated per
/// machine (see `LedConfig`). Defaults are chosen so, unflipped, the newest
/// sample is at the physical top and the wattage bar grows from column 0.
#[derive(Clone, Copy)]
pub struct Orient {
    /// Reverse the time axis (put the newest sample at the bottom instead).
    pub flip_time: bool,
    /// Reverse the wattage bar's growth direction along the short axis.
    pub flip_watts: bool,
}

/// Rolling per-panel wattage history: `frac[0]` is the newest sample, `frac[i]`
/// is `i` samples old. Fixed [`HEIGHT`] deep (one slot per time row); slots not
/// yet filled read 0.0 (dark).
pub struct History {
    frac: [f64; HEIGHT],
}

impl Default for History {
    fn default() -> Self {
        History {
            frac: [0.0; HEIGHT],
        }
    }
}

impl History {
    pub fn new() -> Self {
        History::default()
    }

    /// Records the newest fill fraction, scrolling older samples one row toward
    /// the oldest end and dropping the eldest. Non-finite / out-of-range inputs
    /// are clamped to `[0.0, 1.0]`.
    pub fn push(&mut self, fraction: f64) {
        self.frac.copy_within(0..HEIGHT - 1, 1);
        self.frac[0] = if fraction.is_finite() {
            fraction.clamp(0.0, 1.0)
        } else {
            0.0
        };
    }
}

/// Renders the history into a `[column][row]` grid of grayscale bytes ready for
/// `SendCol` (grid[x] is column x's 34 values). Each time row draws a
/// horizontal bar of width `frac * WIDTH`; the single boundary pixel is dimmed
/// by the sub-pixel remainder so the bar edge moves smoothly across the coarse
/// 9-wide axis. `on` is the lit-pixel level.
pub fn grid(history: &History, on: u8, orient: Orient) -> [[u8; HEIGHT]; WIDTH] {
    let mut g = [[0u8; HEIGHT]; WIDTH];
    for (t, &f) in history.frac.iter().enumerate() {
        let lit = f * WIDTH as f64;
        let full = lit.floor() as usize; // fully-lit columns
        let boundary = lit - full as f64; // partial edge column [0, 1)
        // Time index t (0 = newest) -> physical row. Within a column, index 0
        // is the panel's physical top (calibrated), so newest-on-top is the
        // unflipped mapping.
        let row = if orient.flip_time { HEIGHT - 1 - t } else { t };
        for x in 0..WIDTH {
            let level = if x < full {
                on
            } else if x == full && full < WIDTH {
                (on as f64 * boundary).round() as u8
            } else {
                0
            };
            let col = if orient.flip_watts { WIDTH - 1 - x } else { x };
            g[col][row] = level;
        }
    }
    g
}

#[cfg(test)]
mod tests {
    use super::*;

    const NO_FLIP: Orient = Orient {
        flip_time: false,
        flip_watts: false,
    };

    #[test]
    fn history_scrolls_newest_first() {
        let mut h = History::new();
        h.push(0.1);
        h.push(0.2);
        h.push(0.3);
        assert_eq!(h.frac[0], 0.3, "newest at index 0");
        assert_eq!(h.frac[1], 0.2);
        assert_eq!(h.frac[2], 0.1);
        assert_eq!(h.frac[3], 0.0, "unfilled slots stay dark");
    }

    #[test]
    fn history_drops_eldest_past_capacity() {
        let mut h = History::new();
        for i in 0..HEIGHT + 5 {
            h.push(i as f64 / 100.0);
        }
        // Newest is the last pushed; the eldest survivor is HEIGHT-1 old.
        assert_eq!(h.frac[0], (HEIGHT + 4) as f64 / 100.0);
        assert_eq!(h.frac[HEIGHT - 1], 5.0 / 100.0);
    }

    #[test]
    fn history_clamps_bad_input() {
        let mut h = History::new();
        h.push(-1.0);
        h.push(2.0);
        h.push(f64::NAN);
        assert_eq!(h.frac[0], 0.0); // NaN -> dark
        assert_eq!(h.frac[1], 1.0); // clamped up
        assert_eq!(h.frac[2], 0.0); // clamped down
    }

    #[test]
    fn newest_sample_is_top_row_unflipped() {
        let mut h = History::new();
        h.push(1.0); // full-width bar, newest
        let g = grid(&h, 255, NO_FLIP);
        // Row 0 (physical top) is fully lit across all columns...
        assert!((0..WIDTH).all(|x| g[x][0] == 255));
        // ...and the next row (older, empty) is dark.
        assert!((0..WIDTH).all(|x| g[x][1] == 0));
    }

    #[test]
    fn wattage_is_a_horizontal_bar_from_column_zero() {
        let mut h = History::new();
        h.push(3.0 / WIDTH as f64); // exactly 3 columns lit
        let g = grid(&h, 200, NO_FLIP);
        assert_eq!(g[0][0], 200);
        assert_eq!(g[1][0], 200);
        assert_eq!(g[2][0], 200);
        assert_eq!(g[3][0], 0, "4th column dark");
    }

    #[test]
    fn boundary_column_is_dimmed() {
        let mut h = History::new();
        h.push(1.5 / WIDTH as f64); // 1 full column + half of the next
        let g = grid(&h, 200, NO_FLIP);
        assert_eq!(g[0][0], 200);
        assert_eq!(g[1][0], 100); // 200 * 0.5
        assert_eq!(g[2][0], 0);
    }

    #[test]
    fn flip_time_moves_newest_to_bottom() {
        let mut h = History::new();
        h.push(1.0);
        let g = grid(
            &h,
            255,
            Orient {
                flip_time: true,
                flip_watts: false,
            },
        );
        assert_eq!(g[0][HEIGHT - 1], 255, "newest at physical bottom");
        assert_eq!(g[0][0], 0);
    }

    #[test]
    fn flip_watts_grows_bar_from_the_other_end() {
        let mut h = History::new();
        h.push(2.0 / WIDTH as f64); // 2 columns lit
        let g = grid(
            &h,
            255,
            Orient {
                flip_time: false,
                flip_watts: true,
            },
        );
        assert_eq!(g[WIDTH - 1][0], 255);
        assert_eq!(g[WIDTH - 2][0], 255);
        assert_eq!(g[WIDTH - 3][0], 0);
    }
}
