//! Pure software board renderer: rasterizes a board (outline, pads, traces, vias) into
//! an RGBA pixel buffer using the same ViewTransform the egui canvas uses. This makes
//! the render path verifiable headlessly (no winit/GL window needed) and produces a
//! real image artifact - the GUI's interactive canvas and this renderer share the same
//! geometry math (view.rs), so a correct image here means the canvas draws correctly too.

use fr_board::Board;
use fr_geometry::{IntBox, Point};

use crate::view::ViewTransform;

/// A simple RGBA8 image buffer.
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub px: Vec<[u8; 4]>,
}

impl Image {
    pub fn new(width: u32, height: u32, bg: [u8; 4]) -> Image {
        Image { width, height, px: vec![bg; (width * height) as usize] }
    }

    fn set(&mut self, x: i32, y: i32, c: [u8; 4]) {
        if x >= 0 && y >= 0 && (x as u32) < self.width && (y as u32) < self.height {
            self.px[(y as u32 * self.width + x as u32) as usize] = c;
        }
    }

    /// Count pixels not equal to `bg` (used by tests to confirm something was drawn).
    pub fn non_bg_count(&self, bg: [u8; 4]) -> usize {
        self.px.iter().filter(|p| **p != bg).count()
    }

    /// Encode as a minimal binary PPM (P6) - dependency-free image output for artifacts.
    pub fn to_ppm(&self) -> Vec<u8> {
        let mut out = format!("P6\n{} {}\n255\n", self.width, self.height).into_bytes();
        out.reserve((self.width * self.height * 3) as usize);
        for p in &self.px {
            out.push(p[0]);
            out.push(p[1]);
            out.push(p[2]);
        }
        out
    }
}

fn layer_color(layer: usize) -> [u8; 4] {
    const PALETTE: [[u8; 4]; 6] = [
        [220, 60, 60, 255], [60, 140, 220, 255], [80, 200, 100, 255],
        [220, 180, 60, 255], [200, 100, 220, 255], [60, 200, 200, 255],
    ];
    PALETTE[layer % PALETTE.len()]
}

/// Render the board into a new image of the given size.
pub fn render_board(board: &Board, width: u32, height: u32) -> Image {
    let bg = [20, 24, 20, 255];
    let mut img = Image::new(width, height, bg);

    let bounds = board
        .outline_box()
        .or_else(|| IntBox::bound(board.pins.iter().map(|p| p.location)))
        .unwrap_or(IntBox::new(0, 0, 1, 1));
    let screen = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(width as f32, height as f32));
    let vt = ViewTransform::fit(bounds, screen);
    let to_px = |p: Point| {
        let s = vt.to_screen(p, screen);
        (s.x.round() as i32, s.y.round() as i32)
    };

    // board outline
    if board.outline.len() >= 2 {
        for i in 0..board.outline.len() {
            let a = to_px(board.outline[i]);
            let b = to_px(board.outline[(i + 1) % board.outline.len()]);
            draw_line(&mut img, a, b, [160, 160, 160, 255]);
        }
    }
    // pads
    for pin in &board.pins {
        let (x, y) = to_px(pin.location);
        draw_dot(&mut img, x, y, 1, [110, 110, 110, 255]);
    }
    // traces (per-layer color)
    for t in &board.traces {
        let col = layer_color(t.layer);
        for seg in t.corners.windows(2) {
            draw_line(&mut img, to_px(seg[0]), to_px(seg[1]), col);
        }
    }
    // vias
    for v in &board.vias {
        let (x, y) = to_px(v.location);
        draw_dot(&mut img, x, y, 2, [255, 255, 255, 255]);
    }
    img
}

fn draw_dot(img: &mut Image, cx: i32, cy: i32, r: i32, c: [u8; 4]) {
    for dy in -r..=r {
        for dx in -r..=r {
            img.set(cx + dx, cy + dy, c);
        }
    }
}

/// Bresenham line.
fn draw_line(img: &mut Image, a: (i32, i32), b: (i32, i32), c: [u8; 4]) {
    let (mut x0, mut y0) = a;
    let (x1, y1) = b;
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    loop {
        img.set(x0, y0, c);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{FixedState, Layer, LayerStack, Resolution, Trace, Unit};

    fn board_with_trace() -> Board {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![Layer { name: "Top".into(), index: 0, is_signal: true, preferred: None }]);
        b.outline = vec![
            Point::new(0, 0), Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000), Point::new(0, 1_000_000),
        ];
        b.traces.push(Trace {
            layer: 0, width: 50_000,
            corners: vec![Point::new(100_000, 500_000), Point::new(900_000, 500_000)],
            net: Some(0), fixed: FixedState::Route,
        });
        b
    }

    #[test]
    fn renders_non_blank_image() {
        let bg = [20, 24, 20, 255];
        let img = render_board(&board_with_trace(), 400, 300);
        assert_eq!(img.width, 400);
        // the outline + trace must have drawn many non-background pixels
        assert!(img.non_bg_count(bg) > 200, "expected a drawn board, got {} px", img.non_bg_count(bg));
    }

    #[test]
    fn ppm_header_and_size() {
        let img = render_board(&board_with_trace(), 64, 48);
        let ppm = img.to_ppm();
        assert!(ppm.starts_with(b"P6\n64 48\n255\n"));
        assert_eq!(ppm.len(), "P6\n64 48\n255\n".len() + 64 * 48 * 3);
    }
}
