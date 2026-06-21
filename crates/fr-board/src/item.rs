//! Routable / placed board items: pins, traces (wires), and vias.

use fr_geometry::Point;

/// How firmly an item is fixed (affects whether the router may move/rip it up, and the
/// (type ...) tag written to SES/RTE).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FixedState {
    /// Freely routable / rippable (autorouted result). Written as (type route).
    Route,
    /// User-protected; not altered unless unprotected. Written as (type protect).
    Protect,
    /// System-fixed (pads, locked). Written as (type fix).
    Fix,
}

impl FixedState {
    pub fn type_token(self) -> &'static str {
        match self {
            FixedState::Route => "route",
            FixedState::Protect => "protect",
            FixedState::Fix => "fix",
        }
    }
}

/// A pin: a component's connection point, with copper from a padstack.
#[derive(Clone, Debug)]
pub struct Pin {
    pub component: String,
    pub name: String,
    pub padstack: usize,
    pub location: Point,
    pub net: Option<usize>,
}

/// A routed trace: a polyline on one layer with a width, belonging to a net.
#[derive(Clone, Debug)]
pub struct Trace {
    pub layer: usize,
    pub width: i64,
    pub corners: Vec<Point>,
    pub net: Option<usize>,
    pub fixed: FixedState,
}

/// A via at a point, connecting layers via a named padstack, belonging to a net.
#[derive(Clone, Debug)]
pub struct Via {
    pub padstack: usize,
    pub location: Point,
    pub net: Option<usize>,
    pub fixed: FixedState,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_tokens() {
        assert_eq!(FixedState::Route.type_token(), "route");
        assert_eq!(FixedState::Protect.type_token(), "protect");
        assert_eq!(FixedState::Fix.type_token(), "fix");
    }
}
