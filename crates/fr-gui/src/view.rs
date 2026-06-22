//! Board view state: pan/zoom transform between board units and screen pixels.

use egui::{Pos2, Rect, Vec2};
use fr_geometry::{IntBox, Point};

/// Maps board-unit coordinates to screen pixels (pan + zoom). Board Y is up; screen Y
/// is down, so the transform flips Y.
#[derive(Clone, Copy, Debug)]
pub struct ViewTransform {
    /// pixels per board unit
    pub scale: f64,
    /// board-unit point currently at the screen-rect center
    pub center: Point,
}

impl ViewTransform {
    /// Fit `bounds` into `screen` with a small margin.
    pub fn fit(bounds: IntBox, screen: Rect) -> ViewTransform {
        let w = bounds.width().max(1) as f64;
        let h = bounds.height().max(1) as f64;
        let sx = screen.width() as f64 / w;
        let sy = screen.height() as f64 / h;
        let scale = sx.min(sy) * 0.9;
        let center = Point::new((bounds.ll.x + bounds.ur.x) / 2, (bounds.ll.y + bounds.ur.y) / 2);
        ViewTransform { scale: if scale > 0.0 { scale } else { 1e-6 }, center }
    }

    pub fn to_screen(&self, p: Point, screen: Rect) -> Pos2 {
        let dx = (p.x - self.center.x) as f64 * self.scale;
        let dy = (p.y - self.center.y) as f64 * self.scale;
        Pos2::new(
            screen.center().x + dx as f32,
            // flip Y: board up -> screen down
            screen.center().y - dy as f32,
        )
    }

    pub fn to_board(&self, s: Pos2, screen: Rect) -> Point {
        let dx = (s.x - screen.center().x) as f64 / self.scale;
        let dy = (screen.center().y - s.y) as f64 / self.scale;
        Point::new(self.center.x + dx as i64, self.center.y + dy as i64)
    }

    /// Zoom by `factor` keeping the board point under `anchor` fixed on screen.
    pub fn zoom_at(&mut self, factor: f64, anchor: Pos2, screen: Rect) {
        let before = self.to_board(anchor, screen);
        self.scale = (self.scale * factor).clamp(1e-9, 1e3);
        let after = self.to_board(anchor, screen);
        // shift center so the anchor board-point stays put
        self.center = Point::new(
            self.center.x + (before.x - after.x),
            self.center.y + (before.y - after.y),
        );
    }

    /// Pan by a screen-pixel delta.
    pub fn pan_pixels(&mut self, delta: Vec2) {
        self.center = Point::new(
            self.center.x - (delta.x as f64 / self.scale) as i64,
            self.center.y + (delta.y as f64 / self.scale) as i64,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn screen_board_roundtrip() {
        let bounds = IntBox::new(0, 0, 1_000_000, 1_000_000);
        let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 600.0));
        let vt = ViewTransform::fit(bounds, screen);
        let p = Point::new(250_000, 750_000);
        let s = vt.to_screen(p, screen);
        let back = vt.to_board(s, screen);
        // round-trip within a board unit or two (f32 screen precision)
        assert!((back.x - p.x).abs() < 2000);
        assert!((back.y - p.y).abs() < 2000);
    }
}
