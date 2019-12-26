#![warn(
	clippy::all,
	// clippy::restriction,
	clippy::pedantic,
	clippy::nursery,
	clippy::cargo
)]
#![allow(dead_code)]

use std::collections::{HashMap, HashSet};
use std::hash::Hash;

pub trait Vertex2D {
	/// The coordinate pair of this vertex. In a simple square grid, it could
	/// be expected to return a (x, y) pair.
	fn coords(&self) -> (f64, f64);

	/// Returns the euclidean distance between two vertices, defined as the
	/// length of a straight line connecting them, i.e., `dist = √(Δx² + Δy²)`.
	fn euclidean_distance(&self, other: &Self) -> f64 {
		let (x, y) = self.coords();
		let (xf, yf) = other.coords();
		let x_dist = xf - x;
		let y_dist = yf - y;
		(x_dist.powf(2.0) + y_dist.powf(2.0)).sqrt()
	}

	/// Returns the largest axial distance between two vertices, i.e.,
	/// `dist = max(Δx, Δy)`.
	fn chebyshev_distance(&self, other: &Self) -> f64 {
		let (x, y) = self.coords();
		let (xf, yf) = other.coords();
		let x_dist = (xf - x).abs();
		let y_dist = (yf - y).abs();
		x_dist.max(y_dist)
	}

	/// Returns the distance between two vertices as if they were in a square
	/// grid and diagonal movement were disallowed, i.e., `dist = Δx + Δy`.
	fn manhattan_distance(&self, other: &Self) -> f64 {
		let (x, y) = self.coords();
		let (xf, yf) = other.coords();
		let x_dist = (xf - x).abs();
		let y_dist = (yf - y).abs();
		x_dist + y_dist
	}
}

pub trait Graph2D<V>
where
	V: Vertex2D,
{
	/// Returns all neighboring nodes to a given vertex.
	fn neighbors(&self, vertex: &V) -> Vec<&V>;

	/// Determines whether it is possible to travel between vertices.
	fn path_is_transversable(&self, vertex: &V, other: &V) -> bool;

	/// Returns whether or not the graph has a given vertex.
	fn has_vertex(&self, vertex: &V) -> bool;

	/// Returns the estimated cost of transversing the graph from one vertex to
	/// a goal.
	fn heuristic(&self, vertex: &V, other: &V) -> f64;

	/// Returns the exact cost of transversing the graph from one vertex to
	/// its neighbor.
	fn travel_cost(&self, vertex: &V, other: &V) -> f64;
}

struct NodeInfo<N> {
	parent: Option<N>,
	g_score: f64,
	f_score: f64,
}

impl<N> Default for NodeInfo<N> {
	fn default() -> Self {
		const INF: f64 = std::f64::INFINITY;
		Self {
			parent: None,
			g_score: INF,
			f_score: INF,
		}
	}
}

/// The A* algorithm calculates a path between two points on a graph by picking
/// the points in it that are connected by the lowest costs, `f`, to reach the
/// final goal. This score is defined as the sum of the cost of moving between
/// vertices, `g`, and the estimated cost of reaching the goal from any
/// vertex, `h` - which is called a heuristic function -; i.e. `f = g + h`.
///
/// This function assumes these as properties of the searched surface in
/// relation to its inner points, and leaves their implementation to the user,
/// as it is impossible to define any two generic `g` and `h` functions.
///
/// There is no inherit global guarantee of speed or perfectness, these are
/// highly dependent on the `g` and `h` of choice.
/// If no path is found between the start and finish points, `None` is returned.
pub fn a_star<'m, G, V>(map: &'m G, start: &'m V, goal: &'m V) -> Option<Vec<&'m V>>
where
	G: Graph2D<V>,
	V: Hash + Eq + Vertex2D,
{
	// Estimate how many nodes need to be visited between start and goal
	// to lower the number of allocations required
	let (start_x, start_y) = start.coords();
	let (goal_x, goal_y) = goal.coords();
	let dx = (goal_x - start_x).abs().max(1.0);
	let dy = (goal_y - start_y).abs().max(1.0);
	let area = dx * dy;
	let perimeter = 2.0 * (dx + dy);
	let estimated_visits = if area.is_normal() { area as usize } else { 0 };
	let estimated_analysis = if perimeter.is_normal() {
		perimeter as usize
	} else {
		0
	};
	let mut path = Vec::new();
	let mut open_list = HashSet::<&V>::with_capacity(estimated_analysis);
	open_list.insert(start);
	let mut node_info = HashMap::<&V, NodeInfo<&V>>::with_capacity(estimated_visits);
	// The start node has a zero cost to move to, and given f = g + h,
	// initialize that score to the heuristics alone
	node_info.insert(
		start,
		NodeInfo {
			parent: None,
			g_score: 0.0,
			f_score: map.heuristic(start, goal),
		},
	);
	// Exhaust all pathing possibilities
	'list: while !open_list.is_empty() {
		let mut list = open_list.iter();
		let cmp_node = list.next().unwrap();
		// Pick the node with the lowest f score for analysis
		let mut cur_node = list.fold(*cmp_node, |acc, node| {
			let f_acc = node_info.entry(acc).or_default().f_score;
			let f_node = node_info.entry(node).or_default().f_score;
			if f_acc < f_node {
				acc
			} else {
				*node
			}
		});
		open_list.remove(cur_node);
		for neighbor in &map.neighbors(cur_node) {
			// Check its neighbors for walkability and how good of a pathing
			// choice it is
			if !map.path_is_transversable(cur_node, *neighbor) {
				continue;
			}
			if **neighbor == *goal {
				// We're done! Reconstruct the path
				path.push(cur_node);
				while node_info.contains_key(cur_node) {
					if let Some(parent) = node_info[cur_node].parent {
						cur_node = parent;
						path.push(cur_node);
					} else {
						// We reached the starting node
						break;
					}
				}
				// Since the path is built from last to finish, we must reverse
				// it before pushing the goal and returning it.
				path.reverse();
				path.push(goal);
				break 'list;
			}
			let new_g = node_info.entry(cur_node).or_default().g_score
				+ map.travel_cost(cur_node, neighbor);
			let neighbor_info = node_info.entry(neighbor).or_default();
			if neighbor_info.g_score > new_g {
				let new_f = new_g + map.heuristic(neighbor, goal);
				neighbor_info.f_score = new_f;
				neighbor_info.g_score = new_g;
				neighbor_info.parent = Some(cur_node);
				open_list.insert(neighbor);
			}
		}
	}
	if path.is_empty() {
		None
	} else {
		// println!(
		// 	"{} {} / {} {}",
		// 	estimated_visits,
		// 	node_info.len(),
		// 	estimated_analysis,
		// 	open_list.capacity()
		// );
		Some(path)
	}
}
