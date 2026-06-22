//! fr-spatial: exact-geometry obstacle index over the board's copper.
//!
//! The grid A* checks passability only at node CENTERS, so a trace segment running
//! between two passable nodes can still sweep through a different-net pad or trace —
//! this is the source of the residual trace-to-pad shorts the grid router cannot avoid.
//! This index represents the true copper geometry (pads/vias as discs, traces as
//! fattened segments) in an `rstar` R-tree per layer, and answers the exact question the
//! room/door search needs: *does a proposed trace segment keep `clearance` from every
//! piece of DIFFERENT-net copper?*
//!
//! Distances use the shared `fr-geometry` kernel. The R-tree is an AABB pre-filter; the
//! decision is always made by the exact segment/point distance test, so the answer is
//! conservative-free (no false clears).

use fr_geometry::{point_seg_dist, seg_seg_dist, IntBox, Point};
use rstar::{RTree, RTreeObject, AABB};

/// A single piece of copper on one layer, tagged with its owning net.
#[derive(Clone, Debug)]
enum Shape {
    /// Circular copper (pad or via) of `radius` centered at `center`.
    Disc { center: Point, radius: i64 },
    /// A trace segment `a`-`b` with the given `half` width.
    Seg { a: Point, b: Point, half: i64 },
}

impl Shape {
    /// Minimum distance from this shape's CENTERLINE/center to the segment `a`-`b`.
    fn dist_to_seg(&self, a: Point, b: Point) -> f64 {
        match *self {
            Shape::Disc { center, .. } => point_seg_dist(center, a, b),
            Shape::Seg { a: sa, b: sb, .. } => seg_seg_dist(sa, sb, a, b),
        }
    }
    /// The copper half-extent added to a centerline distance: disc radius or seg half.
    fn half_extent(&self) -> i64 {
        match *self {
            Shape::Disc { radius, .. } => radius,
            Shape::Seg { half, .. } => half,
        }
    }
    /// Axis-aligned bounding box of the actual copper (centerline grown by half-extent).
    fn bbox(&self) -> IntBox {
        match *self {
            Shape::Disc { center, radius } => {
                IntBox::new(center.x - radius, center.y - radius, center.x + radius, center.y + radius)
            }
            Shape::Seg { a, b, half } => IntBox::from_points(a, b).offset(half),
        }
    }
}

/// An indexed obstacle: a shape, its owning net, and a cached AABB envelope.
#[derive(Clone, Debug)]
struct Obstacle {
    shape: Shape,
    net: u32,
    env: AABB<[i64; 2]>,
}

impl RTreeObject for Obstacle {
    type Envelope = AABB<[i64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.env
    }
}

/// No specific owner (board edge, multi-net junction). Different from every real net id.
pub const NO_NET: u32 = u32::MAX;

/// Per-layer exact obstacle index. A via spans layers, so it is inserted on every layer
/// its padstack covers. Built once, then queried (and optionally appended to) as routing
/// progresses.
pub struct ObstacleIndex {
    layers: usize,
    /// Obstacles staged for bulk-load per layer (drained when the tree is built).
    staged: Vec<Vec<Obstacle>>,
    trees: Vec<RTree<Obstacle>>,
    built: bool,
}

fn aabb_of(b: IntBox) -> AABB<[i64; 2]> {
    AABB::from_corners([b.ll.x, b.ll.y], [b.ur.x, b.ur.y])
}

impl ObstacleIndex {
    /// Create an empty index for `layers` layers.
    pub fn new(layers: usize) -> ObstacleIndex {
        let layers = layers.max(1);
        ObstacleIndex {
            layers,
            staged: vec![Vec::new(); layers],
            trees: Vec::new(),
            built: false,
        }
    }

    pub fn layers(&self) -> usize {
        self.layers
    }

    fn push(&mut self, layer: usize, shape: Shape, net: u32) {
        if layer >= self.layers {
            return;
        }
        let env = aabb_of(shape.bbox());
        let obs = Obstacle { shape, net, env };
        if self.built {
            self.trees[layer].insert(obs);
        } else {
            self.staged[layer].push(obs);
        }
    }

    /// Add a circular pad/via disc on `layer`.
    pub fn add_disc(&mut self, layer: usize, center: Point, radius: i64, net: u32) {
        self.push(layer, Shape::Disc { center, radius }, net);
    }

    /// Add a via disc on every layer in `[lo, hi]` (inclusive).
    pub fn add_via(&mut self, lo: usize, hi: usize, center: Point, radius: i64, net: u32) {
        for layer in lo..=hi.min(self.layers - 1) {
            self.add_disc(layer, center, radius, net);
        }
    }

    /// Add a trace polyline on `layer` as a chain of fattened segments.
    pub fn add_trace(&mut self, layer: usize, corners: &[Point], width: i64, net: u32) {
        let half = width / 2;
        for w in corners.windows(2) {
            self.push(layer, Shape::Seg { a: w[0], b: w[1], half }, net);
        }
    }

    /// Finalize staged obstacles into bulk-loaded R-trees. Cheaper and better-balanced
    /// than inserting one by one; call once after the initial board population. Further
    /// `add_*` calls insert incrementally into the live trees.
    pub fn build(&mut self) {
        if self.built {
            return;
        }
        self.trees = self
            .staged
            .iter_mut()
            .map(|v| RTree::bulk_load(std::mem::take(v)))
            .collect();
        self.built = true;
    }

    /// True if a trace segment `a`-`b` on `layer` with half-width `half`, belonging to
    /// `net`, keeps at least `clearance` from every DIFFERENT-net obstacle. Same-net
    /// copper is ignored (a net may touch itself). `NO_NET` obstacles (board edge) block
    /// everyone.
    ///
    /// The decision is exact: copper-to-copper distance is
    /// `dist(centerlines) - half - obstacle_half`, which must be `>= clearance`.
    pub fn segment_is_clear(
        &self,
        layer: usize,
        a: Point,
        b: Point,
        half: i64,
        net: u32,
        clearance: i64,
    ) -> bool {
        // Grow the search by `clearance` so an obstacle within clearance (but not
        // touching the segment's copper bbox) is still a candidate.
        self.min_clearance_margin_within(layer, a, b, half, net, clearance.max(0))
            .map(|m| m >= clearance as f64)
            .unwrap_or(true)
    }

    /// The smallest copper gap (in board units) between the segment and any different-net
    /// obstacle on `layer`, or None if there is no such obstacle within the touching
    /// envelope. Negative means the copper overlaps. This searches only obstacles whose
    /// copper bbox touches the segment copper bbox (gap <= 0); use
    /// `min_clearance_margin_within` to also find near-but-not-touching obstacles.
    pub fn min_clearance_margin(
        &self,
        layer: usize,
        a: Point,
        b: Point,
        half: i64,
        net: u32,
    ) -> Option<f64> {
        self.min_clearance_margin_within(layer, a, b, half, net, 0)
    }

    /// Like `min_clearance_margin`, but grows the query envelope by `search` board units
    /// so obstacles up to `search` away (copper-to-copper) are also considered. Returns
    /// None only if no different-net obstacle lies within the grown envelope.
    pub fn min_clearance_margin_within(
        &self,
        layer: usize,
        a: Point,
        b: Point,
        half: i64,
        net: u32,
        search: i64,
    ) -> Option<f64> {
        if !self.built || layer >= self.trees.len() {
            return None;
        }
        // rstar tests against each obstacle's STORED (already-fattened) envelope, so an
        // overlap with this query catches every obstacle whose copper is within `search`
        // of the segment's copper (we grow the segment box by its own half plus `search`).
        let seg_box = IntBox::from_points(a, b).offset(half + search.max(0));
        let query = aabb_of(seg_box);
        let mut best: Option<f64> = None;
        for obs in self.trees[layer].locate_in_envelope_intersecting(&query) {
            if obs.net == net && net != NO_NET {
                continue; // same net may touch
            }
            let center_dist = obs.shape.dist_to_seg(a, b);
            let margin = center_dist - half as f64 - obs.shape.half_extent() as f64;
            best = Some(best.map_or(margin, |m: f64| m.min(margin)));
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idx_with_pad() -> ObstacleIndex {
        let mut idx = ObstacleIndex::new(1);
        // a net-1 pad of radius 50 at (500,500)
        idx.add_disc(0, Point::new(500, 500), 50, 1);
        idx.build();
        idx
    }

    #[test]
    fn segment_through_a_pad_is_not_clear() {
        let idx = idx_with_pad();
        // net-0 trace (half-width 10) running straight through the pad center
        let clear = idx.segment_is_clear(0, Point::new(0, 500), Point::new(1000, 500), 10, 0, 5);
        assert!(!clear, "a trace crossing a different-net pad must NOT be clear");
    }

    #[test]
    fn segment_clear_of_a_pad() {
        let idx = idx_with_pad();
        // trace far below the pad: gap = 500-50-10 = 440 >> clearance
        let clear = idx.segment_is_clear(0, Point::new(0, 0), Point::new(1000, 0), 10, 0, 5);
        assert!(clear, "a far trace is clear");
    }

    #[test]
    fn same_net_may_touch() {
        let idx = idx_with_pad();
        // same net (1) crossing its own pad: allowed
        let clear = idx.segment_is_clear(0, Point::new(0, 500), Point::new(1000, 500), 10, 1, 5);
        assert!(clear, "a net may run through its own pad");
    }

    #[test]
    fn clearance_violation_detected() {
        let idx = idx_with_pad();
        // pad edge at y=550; trace half 10 => centerline at 550+10+8 = 568 leaves gap 8
        let clear = idx.segment_is_clear(0, Point::new(0, 568), Point::new(1000, 568), 10, 0, 20);
        assert!(!clear, "gap 8 < clearance 20 must fail");
        let clear5 = idx.segment_is_clear(0, Point::new(0, 568), Point::new(1000, 568), 10, 0, 5);
        assert!(clear5, "gap 8 >= clearance 5 passes");
    }

    #[test]
    fn diagonal_segment_through_pad() {
        // regression for the exact bug: a diagonal move whose body clips the pad copper.
        let mut idx = ObstacleIndex::new(1);
        idx.add_disc(0, Point::new(100, 100), 30, 1);
        idx.build();
        // diagonal (0,200)->(200,0) lies on x+y=200, which passes through (100,100)
        let clear = idx.segment_is_clear(0, Point::new(0, 200), Point::new(200, 0), 5, 0, 1);
        assert!(!clear, "diagonal through pad center must be caught");
    }

    #[test]
    fn no_obstacle_means_clear() {
        let mut idx = ObstacleIndex::new(2);
        idx.build();
        assert!(idx.segment_is_clear(0, Point::new(0, 0), Point::new(10, 0), 5, 0, 5));
        assert!(idx.min_clearance_margin(0, Point::new(0, 0), Point::new(10, 0), 5, 0).is_none());
    }

    #[test]
    fn incremental_insert_after_build() {
        let mut idx = ObstacleIndex::new(1);
        idx.build(); // empty
        idx.add_trace(0, &[Point::new(0, 100), Point::new(1000, 100)], 20, 2);
        // now a net-0 trace crossing it is blocked
        assert!(!idx.segment_is_clear(0, Point::new(500, 0), Point::new(500, 1000), 10, 0, 5));
    }

    #[test]
    fn via_spans_layers() {
        let mut idx = ObstacleIndex::new(3);
        idx.add_via(0, 2, Point::new(0, 0), 40, 7);
        idx.build();
        for l in 0..3 {
            assert!(!idx.segment_is_clear(l, Point::new(-100, 0), Point::new(100, 0), 5, 0, 1),
                "via blocks every layer it spans (layer {l})");
        }
    }
}
