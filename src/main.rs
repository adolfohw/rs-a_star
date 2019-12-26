#![warn(
	clippy::all,
	// clippy::restriction,
	clippy::pedantic,
	clippy::nursery,
	clippy::cargo
)]

// This is an RLS issue with lib/main crates and this is the workaround
// https://github.com/rust-lang/rls-vscode/issues/686
mod lib;
use lib::*;

use std::fmt::Debug;
use std::ops::Deref;
use std::ops::DerefMut;

#[derive(Clone, Default, PartialEq, Eq, Hash)]
struct Node {
	x: usize,
	y: usize,
	is_wall: bool,
}

impl Debug for Node {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		let desc = if self.is_wall { "Wall" } else { "Free" };
		write!(f, "{}({}, {})", desc, self.x, self.y)
	}
}

impl Vertex2D for Node {
	fn coords(&self) -> (f64, f64) {
		(self.x as f64, self.y as f64)
	}
}

impl Node {
	const fn new(x: usize, y: usize) -> Self {
		Self {
			x,
			y,
			is_wall: false,
		}
	}
}

struct D2Q9 {
	grid: Vec<Vec<Node>>,
}

impl Deref for D2Q9 {
	type Target = [Vec<Node>];
	fn deref(&self) -> &Self::Target {
		Deref::deref(&self.grid)
	}
}

impl DerefMut for D2Q9 {
	fn deref_mut(&mut self) -> &mut Self::Target {
		DerefMut::deref_mut(&mut self.grid)
	}
}

impl D2Q9 {
	fn new<F>(width: usize, height: usize, is_wall: F) -> Self
	where
		F: Fn(usize, usize) -> bool,
	{
		let mut grid = Vec::with_capacity(height);
		for y in 0..height {
			let mut points = Vec::with_capacity(width);
			for x in 0..width {
				points.push(Node {
					x,
					y,
					is_wall: is_wall(x, y),
				});
			}
			grid.push(points);
		}
		Self { grid }
	}

	fn get_at(&self, x: usize, y: usize) -> Option<&Node> {
		self.get(y).map(|row| row.get(x)).flatten()
	}
}

impl Graph2D<Node> for D2Q9 {
	fn has_vertex(&self, node: &Node) -> bool {
		self.get_at(node.x, node.y).is_some()
	}
	fn neighbors(&self, from: &Node) -> Vec<&Node> {
		macro_rules! resolve_or_continue {
			($axis:expr => $delta:ident) => {
				match $axis
					.checked_add($delta)
					.map(|coord| coord.checked_sub(1))
					.flatten()
					{
					None => continue,
					Some(coord) => coord,
					}
			};
			(@ $x:ident $y:ident) => {
				match self.get($y).map(|row| row.get($x)).flatten() {
					None => continue,
					Some(node) => node,
					}
			};
		}
		let mut neighbors = Vec::with_capacity(8);
		for dy in 0..=2 {
			for dx in 0..=2 {
				if dx == dy && dx == 1 {
					continue;
				}
				let x = resolve_or_continue!(from.x => dx);
				let y = resolve_or_continue!(from.y => dy);
				let neighbor = resolve_or_continue!(@ x y);
				neighbors.push(neighbor);
			}
		}
		neighbors
	}

	#[allow(clippy::float_cmp)]
	fn path_is_transversable(&self, from: &Node, to: &Node) -> bool {
		if from == to {
			return true;
		}
		if from.is_wall || to.is_wall {
			return false;
		}
		let dist_x = to.x as f64 - from.x as f64;
		let dist_y = to.y as f64 - from.y as f64;
		if (from.x == to.x && dist_y.abs() == 1.0) || (from.y == to.y && dist_x.abs() == 1.0) {
			return true;
		}
		if dist_x.abs() == dist_y.abs() && dist_x.abs() == 1.0 {
			return Some(false)
				== self
					.get_at(from.x, (from.y as f64 + dist_y) as usize)
					.map(|node| node.is_wall)
				|| Some(false)
					== self
						.get_at((from.x as f64 + dist_x) as usize, from.y)
						.map(|node| node.is_wall);
		}
		false
	}

	fn heuristic(&self, node: &Node, other: &Node) -> f64 {
		node.chebyshev_distance(other)
	}

	fn travel_cost(&self, node: &Node, other: &Node) -> f64 {
		node.euclidean_distance(other)
	}
}

impl D2Q9 {
	fn path_and_show<'p>(&'p self, from: &'p Node, to: &'p Node) -> Option<Vec<&'p Node>> {
		let path = a_star(self, from, to)?;
		for row in self.iter() {
			for node in row.iter() {
				if node.x == from.x && node.y == from.y {
					print!("S")
				} else if node.x == to.x && node.y == to.y {
					print!("E")
				} else if node.is_wall {
					print!("O");
				} else if path.contains(&node) {
					print!("#");
				// } else if visited.contains(node) {
				// print!(".");
				} else {
					print!(" ");
				}
			}
			println!();
		}
		Some(path)
	}
}

fn main() {
	let grid = D2Q9::new(50, 20, |x, y| {
		// (x % 10 == 3 && y > 5) || (x % 10 == 8 && y < 15)
		// x < 48 && x / 5 == y && y < 15
		(x == 5 && y >= 3 && y <= 5)
			|| (x == 30 && y >= 5 && y <= 10)
			|| (x == 35 && y <= 10 && y >= 3)
			|| (y == 3 && x >= 5 && x <= 35)
			|| (y == 5 && x >= 5 && x <= 30)
			|| (y == 10 && x >= 30 && x <= 35)
	});
	grid.path_and_show(&Node::new(0, 19), &Node::new(37, 1));
}
