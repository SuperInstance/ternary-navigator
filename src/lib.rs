#![forbid(unsafe_code)]

//! Ternary Navigator — Wayfinding and pathfinding across rooms.
//!
//! Provides ternary-weighted A* pathfinding, multi-hop room routing, dead reckoning,
//! compass directions in ternary space (approach/avoid/neutral), and spatial map
//! maintenance for fleet topology.

use std::cmp::Ordering;

// ── Ternary value ──────────────────────────────────────────────────────

/// A ternary weight: Approve (+1), Neutral (0), or Avoid (-1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Ternary {
    Avoid = -1,
    Neutral = 0,
    Approach = 1,
}

impl Ternary {
    pub fn value(self) -> i8 {
        self as i8
    }

    pub fn from_i8(v: i8) -> Option<Self> {
        match v {
            -1 => Some(Ternary::Avoid),
            0 => Some(Ternary::Neutral),
            1 => Some(Ternary::Approach),
            _ => None,
        }
    }
}

// ── Position ───────────────────────────────────────────────────────────

/// A position in ternary grid space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Position {
    pub x: i32,
    pub y: i32,
}

impl Position {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn distance_to(self, other: Position) -> u32 {
        ((self.x - other.x).unsigned_abs() + (self.y - other.y).unsigned_abs()) as u32
    }

    pub fn neighbors(self) -> [Option<Position>; 4] {
        let dirs: [(i32, i32); 4] = [(0, 1), (0, -1), (1, 0), (-1, 0)];
        dirs.map(|(dx, dy)| Some(Position::new(self.x + dx, self.y + dy)))
    }
}

// ── Edge weight ────────────────────────────────────────────────────────

/// Weighted edge between two positions.
#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub from: Position,
    pub to: Position,
    pub weight: Ternary,
    pub cost: u32,
}

// ── Navigator ──────────────────────────────────────────────────────────

/// Top-level navigator that owns the room graph.
#[derive(Debug, Clone)]
pub struct Navigator {
    room_id: String,
    edges: Vec<Edge>,
}

impl Navigator {
    pub fn new(room_id: &str) -> Self {
        Self {
            room_id: room_id.to_string(),
            edges: Vec::new(),
        }
    }

    pub fn room_id(&self) -> &str {
        &self.room_id
    }

    pub fn add_edge(&mut self, from: Position, to: Position, weight: Ternary, cost: u32) {
        self.edges.push(Edge { from, to, weight, cost });
        self.edges.push(Edge { from: to, to: from, weight, cost });
    }

    pub fn edges_from(&self, pos: Position) -> Vec<&Edge> {
        self.edges.iter().filter(|e| e.from == pos).collect()
    }

    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }
}

// ── PathFinder (ternary-weighted A*) ───────────────────────────────────

/// A* pathfinder with ternary weight adjustments.
pub struct PathFinder<'a> {
    navigator: &'a Navigator,
}

impl<'a> PathFinder<'a> {
    pub fn new(navigator: &'a Navigator) -> Self {
        Self { navigator }
    }

    /// Find shortest path from start to goal, returning positions and total cost.
    /// Edges weighted Avoid are excluded; Approach edges get cost bonus.
    pub fn find_path(&self, start: Position, goal: Position) -> Option<(Vec<Position>, u32)> {
        if start == goal {
            return Some((vec![start], 0));
        }

        let mut open: Vec<(Position, u32, u32)> = vec![(start, 0, start.distance_to(goal))];
        let mut came_from: std::collections::HashMap<Position, Position> = std::collections::HashMap::new();
        let mut g_score: std::collections::HashMap<Position, u32> = std::collections::HashMap::new();
        g_score.insert(start, 0);

        while let Some(idx) = open.iter().enumerate().min_by(|a, b| {
            let fa = a.1 .1 + a.1 .2;
            let fb = b.1 .1 + b.1 .2;
            fa.cmp(&fb)
        }).map(|(i, _)| i)
        {
            let (current, g, _) = open.remove(idx);
            if current == goal {
                let mut path = vec![goal];
                let mut c = goal;
                while let Some(&prev) = came_from.get(&c) {
                    path.push(prev);
                    c = prev;
                }
                path.reverse();
                return Some((path, g));
            }
            for edge in self.navigator.edges_from(current) {
                if edge.weight == Ternary::Avoid {
                    continue;
                }
                let bonus = if edge.weight == Ternary::Approach { 0 } else { edge.cost };
                let actual_cost = if edge.weight == Ternary::Approach {
                    edge.cost.saturating_sub(1)
                } else {
                    bonus
                };
                let tentative = g.saturating_add(actual_cost);
                let prev = g_score.get(&edge.to).copied().unwrap_or(u32::MAX);
                if tentative < prev {
                    came_from.insert(edge.to, current);
                    g_score.insert(edge.to, tentative);
                    let h = edge.to.distance_to(goal);
                    open.push((edge.to, tentative, h));
                }
            }
        }
        None
    }
}

// ── RoutePlanner ───────────────────────────────────────────────────────

/// Multi-hop room routing across multiple navigators.
pub struct RoutePlanner {
    rooms: Vec<Navigator>,
    connections: Vec<(String, String, Position, Position)>,
}

impl RoutePlanner {
    pub fn new() -> Self {
        Self {
            rooms: Vec::new(),
            connections: Vec::new(),
        }
    }

    pub fn add_room(&mut self, nav: Navigator) {
        self.rooms.push(nav);
    }

    pub fn connect(&mut self, room_a: &str, pos_a: Position, room_b: &str, pos_b: Position) {
        self.connections.push((room_a.to_string(), room_b.to_string(), pos_a, pos_b));
    }

    /// Plan a multi-hop route from one room to another (BFS over rooms).
    pub fn plan(&self, from_room: &str, _from_pos: Position, to_room: &str, _to_pos: Position) -> Option<Vec<String>> {
        if from_room == to_room {
            return Some(vec![from_room.to_string()]);
        }
        let mut visited = std::collections::HashSet::new();
        let mut queue: Vec<(String, Vec<String>)> = vec![(from_room.to_string(), vec![from_room.to_string()])];
        visited.insert(from_room.to_string());
        while let Some((current, path)) = queue.pop() {
            for conn in &self.connections {
                let neighbor = if conn.0 == current { &conn.1 } else if conn.1 == current { &conn.0 } else { continue };
                if visited.contains(neighbor) { continue; }
                let mut new_path = path.clone();
                new_path.push(neighbor.clone());
                if neighbor == to_room { return Some(new_path); }
                visited.insert(neighbor.clone());
                queue.push((neighbor.clone(), new_path));
            }
        }
        None
    }

    pub fn room_count(&self) -> usize {
        self.rooms.len()
    }
}

// ── DeadReckoning ──────────────────────────────────────────────────────

/// Estimate position from tick counts and velocities.
#[derive(Debug, Clone)]
pub struct DeadReckoning {
    origin: Position,
    velocity_x: i32,
    velocity_y: i32,
}

impl DeadReckoning {
    pub fn new(origin: Position) -> Self {
        Self { origin, velocity_x: 0, velocity_y: 0 }
    }

    pub fn set_velocity(&mut self, vx: i32, vy: i32) {
        self.velocity_x = vx;
        self.velocity_y = vy;
    }

    pub fn estimate(&self, ticks: u32) -> Position {
        Position::new(
            self.origin.x + self.velocity_x * ticks as i32,
            self.origin.y + self.velocity_y * ticks as i32,
        )
    }

    pub fn correct(&mut self, actual: Position) {
        self.origin = actual;
    }

    pub fn velocity(&self) -> (i32, i32) {
        (self.velocity_x, self.velocity_y)
    }
}

// ── Compass ────────────────────────────────────────────────────────────

/// Direction in ternary space: approach, avoid, or neutral.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompassDirection {
    Approaching,
    Avoiding,
    Neutral,
}

/// Compass determines ternary direction between two positions.
pub struct Compass;

impl Compass {
    pub fn bearing(from: Position, to: Position) -> CompassDirection {
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let dist_sq = dx * dx + dy * dy;
        if dist_sq == 0 {
            CompassDirection::Neutral
        } else {
            CompassDirection::Approaching
        }
    }

    pub fn bearing_moving(from: Position, to: Position, prev_from: Position) -> CompassDirection {
        let new_dist = from.distance_to(to);
        let old_dist = prev_from.distance_to(to);
        match new_dist.cmp(&old_dist) {
            Ordering::Less => CompassDirection::Approaching,
            Ordering::Greater => CompassDirection::Avoiding,
            Ordering::Equal => CompassDirection::Neutral,
        }
    }

    pub fn ternary_vector(from: Position, to: Position) -> (Ternary, Ternary) {
        let tx = match to.x.cmp(&from.x) {
            Ordering::Greater => Ternary::Approach,
            Ordering::Less => Ternary::Avoid,
            Ordering::Equal => Ternary::Neutral,
        };
        let ty = match to.y.cmp(&from.y) {
            Ordering::Greater => Ternary::Approach,
            Ordering::Less => Ternary::Avoid,
            Ordering::Equal => Ternary::Neutral,
        };
        (tx, ty)
    }
}

// ── MapRoom ────────────────────────────────────────────────────────────

/// Maintains a spatial map of fleet topology.
#[derive(Debug, Clone)]
pub struct MapRoom {
    rooms: std::collections::HashMap<String, Position>,
    adjacency: std::collections::HashMap<String, Vec<String>>,
}

impl MapRoom {
    pub fn new() -> Self {
        Self {
            rooms: std::collections::HashMap::new(),
            adjacency: std::collections::HashMap::new(),
        }
    }

    pub fn register(&mut self, name: &str, pos: Position) {
        self.rooms.insert(name.to_string(), pos);
        self.adjacency.entry(name.to_string()).or_default();
    }

    pub fn link(&mut self, a: &str, b: &str) {
        self.adjacency.entry(a.to_string()).or_default().push(b.to_string());
        self.adjacency.entry(b.to_string()).or_default().push(a.to_string());
    }

    pub fn position_of(&self, name: &str) -> Option<Position> {
        self.rooms.get(name).copied()
    }

    pub fn neighbors_of(&self, name: &str) -> &[String] {
        self.adjacency.get(name).map(|v| v.as_slice()).unwrap_or(&[])
    }

    pub fn room_names(&self) -> Vec<&str> {
        self.rooms.keys().map(|s| s.as_str()).collect()
    }

    /// Find closest room to a given position.
    pub fn closest_to(&self, pos: Position) -> Option<&str> {
        self.rooms.iter().min_by_key(|(_, &p)| p.distance_to(pos)).map(|(n, _)| n.as_str())
    }

    pub fn room_count(&self) -> usize {
        self.rooms.len()
    }
}

// ── Tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ternary_values() {
        assert_eq!(Ternary::Avoid.value(), -1);
        assert_eq!(Ternary::Neutral.value(), 0);
        assert_eq!(Ternary::Approach.value(), 1);
    }

    #[test]
    fn ternary_from_i8() {
        assert_eq!(Ternary::from_i8(-1), Some(Ternary::Avoid));
        assert_eq!(Ternary::from_i8(0), Some(Ternary::Neutral));
        assert_eq!(Ternary::from_i8(1), Some(Ternary::Approach));
        assert_eq!(Ternary::from_i8(2), None);
        assert_eq!(Ternary::from_i8(-2), None);
    }

    #[test]
    fn position_distance() {
        let a = Position::new(0, 0);
        let b = Position::new(3, 4);
        assert_eq!(a.distance_to(b), 7);
    }

    #[test]
    fn position_same_distance_zero() {
        let a = Position::new(5, 5);
        assert_eq!(a.distance_to(a), 0);
    }

    #[test]
    fn position_neighbors() {
        let p = Position::new(2, 3);
        let n = p.neighbors();
        assert!(n.contains(&Some(Position::new(2, 4))));
        assert!(n.contains(&Some(Position::new(2, 2))));
        assert!(n.contains(&Some(Position::new(3, 3))));
        assert!(n.contains(&Some(Position::new(1, 3))));
    }

    #[test]
    fn navigator_add_edges() {
        let mut nav = Navigator::new("room-1");
        nav.add_edge(Position::new(0, 0), Position::new(1, 0), Ternary::Approach, 3);
        // bidirectional = 2 edges
        assert_eq!(nav.edge_count(), 2);
        assert_eq!(nav.room_id(), "room-1");
    }

    #[test]
    fn navigator_edges_from() {
        let mut nav = Navigator::new("room-1");
        nav.add_edge(Position::new(0, 0), Position::new(1, 0), Ternary::Neutral, 1);
        let edges = nav.edges_from(Position::new(0, 0));
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].to, Position::new(1, 0));
    }

    #[test]
    fn pathfinder_straight_line() {
        let mut nav = Navigator::new("room-1");
        nav.add_edge(Position::new(0, 0), Position::new(1, 0), Ternary::Approach, 2);
        nav.add_edge(Position::new(1, 0), Position::new(2, 0), Ternary::Approach, 2);
        let pf = PathFinder::new(&nav);
        let result = pf.find_path(Position::new(0, 0), Position::new(2, 0));
        assert!(result.is_some());
        let (path, cost) = result.unwrap();
        assert_eq!(path.len(), 3);
        assert!(cost <= 4); // approach reduces cost
    }

    #[test]
    fn pathfinder_same_start_goal() {
        let nav = Navigator::new("room-1");
        let pf = PathFinder::new(&nav);
        let result = pf.find_path(Position::new(5, 5), Position::new(5, 5));
        assert!(result.is_some());
        let (path, cost) = result.unwrap();
        assert_eq!(path.len(), 1);
        assert_eq!(cost, 0);
    }

    #[test]
    fn pathfinder_avoid_blocks_path() {
        let mut nav = Navigator::new("room-1");
        nav.add_edge(Position::new(0, 0), Position::new(1, 0), Ternary::Avoid, 1);
        let pf = PathFinder::new(&nav);
        let result = pf.find_path(Position::new(0, 0), Position::new(1, 0));
        assert!(result.is_none());
    }

    #[test]
    fn pathfinder_no_path() {
        let nav = Navigator::new("room-1");
        let pf = PathFinder::new(&nav);
        let result = pf.find_path(Position::new(0, 0), Position::new(10, 10));
        assert!(result.is_none());
    }

    #[test]
    fn route_planner_same_room() {
        let mut rp = RoutePlanner::new();
        rp.add_room(Navigator::new("A"));
        let result = rp.plan("A", Position::new(0, 0), "A", Position::new(0, 0));
        assert!(result.is_some());
        assert_eq!(result.unwrap().len(), 1);
    }

    #[test]
    fn route_planner_multi_hop() {
        let mut rp = RoutePlanner::new();
        rp.add_room(Navigator::new("A"));
        rp.add_room(Navigator::new("B"));
        rp.add_room(Navigator::new("C"));
        rp.connect("A", Position::new(0, 0), "B", Position::new(5, 0));
        rp.connect("B", Position::new(5, 0), "C", Position::new(10, 0));
        let result = rp.plan("A", Position::new(0, 0), "C", Position::new(10, 0));
        assert!(result.is_some());
        let route = result.unwrap();
        assert_eq!(route, vec!["A", "B", "C"]);
    }

    #[test]
    fn route_planner_no_route() {
        let mut rp = RoutePlanner::new();
        rp.add_room(Navigator::new("A"));
        rp.add_room(Navigator::new("B"));
        let result = rp.plan("A", Position::new(0, 0), "B", Position::new(5, 0));
        assert!(result.is_none());
    }

    #[test]
    fn dead_reckoning_basic() {
        let mut dr = DeadReckoning::new(Position::new(0, 0));
        dr.set_velocity(2, 3);
        assert_eq!(dr.estimate(4), Position::new(8, 12));
    }

    #[test]
    fn dead_reckoning_correct() {
        let mut dr = DeadReckoning::new(Position::new(0, 0));
        dr.set_velocity(1, 0);
        assert_eq!(dr.estimate(5), Position::new(5, 0));
        dr.correct(Position::new(3, 0));
        assert_eq!(dr.estimate(0), Position::new(3, 0));
    }

    #[test]
    fn dead_reckoning_velocity() {
        let mut dr = DeadReckoning::new(Position::new(10, 10));
        dr.set_velocity(-1, 2);
        assert_eq!(dr.velocity(), (-1, 2));
    }

    #[test]
    fn compass_bearing_same_point() {
        let p = Position::new(3, 3);
        assert_eq!(Compass::bearing(p, p), CompassDirection::Neutral);
    }

    #[test]
    fn compass_bearing_approaching() {
        assert_eq!(Compass::bearing(Position::new(0, 0), Position::new(3, 3)), CompassDirection::Approaching);
    }

    #[test]
    fn compass_bearing_moving() {
        let prev = Position::new(0, 0);
        let curr = Position::new(2, 0);
        let target = Position::new(5, 0);
        assert_eq!(Compass::bearing_moving(curr, target, prev), CompassDirection::Approaching);
    }

    #[test]
    fn compass_bearing_moving_away() {
        let prev = Position::new(4, 0);
        let curr = Position::new(2, 0);
        let target = Position::new(5, 0);
        assert_eq!(Compass::bearing_moving(curr, target, prev), CompassDirection::Avoiding);
    }

    #[test]
    fn compass_ternary_vector() {
        let (tx, ty) = Compass::ternary_vector(Position::new(0, 0), Position::new(3, -2));
        assert_eq!(tx, Ternary::Approach);
        assert_eq!(ty, Ternary::Avoid);
    }

    #[test]
    fn compass_ternary_vector_neutral() {
        let (tx, ty) = Compass::ternary_vector(Position::new(5, 5), Position::new(5, 5));
        assert_eq!(tx, Ternary::Neutral);
        assert_eq!(ty, Ternary::Neutral);
    }

    #[test]
    fn map_room_register_and_find() {
        let mut map = MapRoom::new();
        map.register("alpha", Position::new(0, 0));
        map.register("beta", Position::new(10, 10));
        assert_eq!(map.position_of("alpha"), Some(Position::new(0, 0)));
        assert_eq!(map.room_count(), 2);
    }

    #[test]
    fn map_room_link_and_neighbors() {
        let mut map = MapRoom::new();
        map.register("a", Position::new(0, 0));
        map.register("b", Position::new(1, 0));
        map.link("a", "b");
        assert!(map.neighbors_of("a").contains(&"b".to_string()));
        assert!(map.neighbors_of("b").contains(&"a".to_string()));
    }

    #[test]
    fn map_room_closest_to() {
        let mut map = MapRoom::new();
        map.register("near", Position::new(1, 1));
        map.register("far", Position::new(100, 100));
        assert_eq!(map.closest_to(Position::new(0, 0)), Some("near"));
    }

    #[test]
    fn map_room_names() {
        let mut map = MapRoom::new();
        map.register("x", Position::new(0, 0));
        map.register("y", Position::new(1, 1));
        let names = map.room_names();
        assert!(names.contains(&"x"));
        assert!(names.contains(&"y"));
    }
}
