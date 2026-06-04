# ternary-navigator

Wayfinding and pathfinding across rooms using ternary-weighted graph traversal.

## Why This Exists

In a fleet of agents distributed across rooms, you need to know how to get from A to B — but not all paths are equal. Some corridors are preferred (approach), some are last resorts (avoid), and some are neutral. Ternary-navigator applies these {-1, 0, +1} weights to A* pathfinding, letting agents naturally prefer or reject routes without arbitrary numeric tuning.

## Core Concepts

- **Ternary weight**: One of three values — Approve (+1), Neutral (0), Avoid (-1). Edges tagged Avoid are excluded from pathfinding entirely; Approve edges get a cost reduction.
- **Room**: A named graph of positions connected by weighted edges.
- **Dead reckoning**: Estimating current position from a known origin and velocity, corrected when ground truth arrives.
- **Compass**: Determining whether movement is approaching, avoiding, or neutral relative to a target, expressed in ternary terms.
- **MapRoom**: A spatial registry of rooms and their interconnections, used for fleet-level topology queries.

## Quick Start

```toml
[dependencies]
ternary-navigator = "0.1"
```

```rust
use ternary_navigator::*;

let mut nav = Navigator::new("bridge");
nav.add_edge(Position::new(0, 0), Position::new(1, 0), Ternary::Approach, 2);
nav.add_edge(Position::new(1, 0), Position::new(2, 0), Ternary::Neutral, 3);

let pf = PathFinder::new(&nav);
if let Some((path, cost)) = pf.find_path(Position::new(0, 0), Position::new(2, 0)) {
    println!("Path: {:?} cost: {}", path, cost);
}
```

## API Overview

| Type | Description |
|------|-------------|
| `Ternary` | Ternary weight enum: Avoid, Neutral, Approach |
| `Position` | (x, y) coordinate in grid space |
| `Edge` | Weighted directed connection between positions |
| `Navigator` | Room graph with edges and path queries |
| `PathFinder` | A* pathfinder with ternary weight adjustments |
| `RoutePlanner` | Multi-hop routing across connected rooms |
| `DeadReckoning` | Position estimator from tick counts and velocity |
| `Compass` | Direction classification in ternary space |
| `MapRoom` | Spatial registry of fleet topology |

## How It Works

`PathFinder` runs A* with a ternary twist: Avoid edges are excluded entirely from the candidate set, while Approach edges have their traversal cost reduced by 1 (flooring at 0). This means preferred routes are genuinely cheaper, not just "less bad." The heuristic is Manhattan distance.

`RoutePlanner` treats rooms as nodes in a higher-level graph and uses BFS to find the shortest sequence of rooms between a source and destination. Individual room-level pathfinding is delegated to `PathFinder`.

`DeadReckoning` is pure linear extrapolation: position = origin + velocity × ticks. When GPS or equivalent arrives, call `correct()` to reset the origin.

`Compass` compares distances between positions to determine ternary direction. `ternary_vector` decomposes a direction into per-axis ternary components.

## Known Limitations

- A* uses a flat Vec for the open set — degrades to O(n²) on dense graphs with thousands of nodes. A binary heap would improve this.
- No diagonal movement support; only 4-connected grids.
- `RoutePlanner` does not consider intra-room distances when choosing between rooms.
- `DeadReckoning` assumes constant velocity between corrections; no acceleration model.
- No serialization support — maps must be reconstructed from code.

## Use Cases

- **Robot fleet routing**: Agents navigate between workstations, preferring known-safe corridors (Approach) and avoiding congestion zones (Avoid).
- **Game AI pathfinding**: NPCs choose paths through a dungeon where some corridors are trapped (Avoid) and others are shortcuts (Approach).
- **Network packet routing**: Route messages through network nodes where some links are high-priority and others should be avoided during congestion.

## Ecosystem Context

Part of the SuperInstance ternary fleet ecosystem. Works alongside `ternary-channel` for messaging between rooms and `ternary-observatory` for monitoring room states. `MapRoom` can be populated from `ternary-beacon` discovery data.

## License

MIT
