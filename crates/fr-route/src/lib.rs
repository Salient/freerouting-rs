//! fr-route: the autorouter search space and single-net router.
//!
//! Phase 5 provides a working router on a uniform grid (deterministic weighted A* with
//! via moves, obstacle clearance, path -> trace/via geometry). The spec's free-angle
//! expansion-room/door model replaces the grid's neighbour generation in a later phase
//! without changing the engine API. Phase 6 adds shove/rip-up and optimization.

pub mod astar;
pub mod grid;
pub mod maze;
pub mod obstacles;
pub mod room;
pub mod router;

pub use astar::{Costs, EdgeValidator};
pub use grid::{Grid, Node};
pub use obstacles::{via_radius, ObstacleMap};
pub use router::{route_connection, RoutedConnection};

// Re-export the exact obstacle index so the engine can build/query it via fr-route.
pub use fr_spatial::{ObstacleIndex, NO_NET};
