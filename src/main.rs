use std::collections::HashMap;
use std::collections::HashSet;
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

fn h(p0: &Node, p: &Node) -> f64 {
	(p.x as f64 - p0.x as f64)
		.abs()
		.max((p.y as f64 - p0.y as f64).abs())
}

fn d(p: &Node, p0: &Node) -> f64 {
	((p.x as f64 - p0.x as f64).powf(2.0) + (p.y as f64 - p0.y as f64).powf(2.0)).sqrt()
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
	fn is_transversable(&self, from: &Node, to: &Node) -> bool {
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
				&& Some(false)
					== self
						.get_at((from.x as f64 + dist_x) as usize, from.y)
						.map(|node| node.is_wall);
		}
		false
	}

	fn path(&self, from: (usize, usize), to: (usize, usize)) -> Vec<Node> {
		const INF: f64 = std::f64::INFINITY;
		let from = Node {
			x: from.0,
			y: from.1,
			is_wall: false,
		};
		let to = Node {
			x: to.0,
			y: to.1,
			is_wall: false,
		};
		let min_nodes = from.x.max(to.x) - from.x.min(to.x);
		let mut path = Vec::new();
		let mut open_list = HashSet::<&Node>::new();
		open_list.insert(&from);
		let mut nodes = HashMap::<&Node, &Node>::with_capacity(min_nodes);
		let mut g_score = HashMap::<&Node, f64>::with_capacity(min_nodes);
		g_score.insert(&from, 0.0);
		let mut f_score = HashMap::<&Node, f64>::with_capacity(min_nodes);
		f_score.insert(&from, h(&from, &to));
		while !open_list.is_empty() {
			let mut list = open_list.iter();
			let cmp_node = list.next().unwrap();
			let cur_node = list.fold(*cmp_node, |acc, node| {
				let f_acc = *f_score.entry(&acc).or_insert(INF);
				let f_node = *f_score.entry(node).or_insert(INF);
				if f_acc < f_node {
					acc
				} else {
					*node
				}
			});
			open_list.remove(cur_node);
			for neighbor in self.neighbors(cur_node).iter() {
				if !self.is_transversable(cur_node, neighbor) {
					continue;
				} else if **neighbor == to {
					let mut node = cur_node;
					path.push(node.clone());
					while nodes.contains_key(node) {
						node = nodes[node];
						path.push(node.clone());
					}
					path.reverse();
					path.push((*neighbor).clone());
					return path;
				}
				let new_g = *g_score.entry(&cur_node).or_insert(INF) + d(&cur_node, neighbor);
				let new_h = h(neighbor, &to);
				let new_f = new_g + new_h;
				let f_neighbor = f_score.entry(neighbor).or_insert(INF);
				if *f_neighbor > new_f {
					*f_neighbor = new_f;
					*g_score.entry(neighbor).or_insert(INF) = new_g;
					*nodes.entry(neighbor).or_insert(&cur_node) = &cur_node;
					open_list.insert(neighbor);
				}
			}
		}
		path
	}

	fn path_and_show(&self, from: (usize, usize), to: (usize, usize)) -> Vec<Node> {
		let path = self.path(from, to);
		for row in self.iter() {
			for node in row.iter() {
				if node.x == from.0 && node.y == from.1 {
					print!("S")
				} else if node.x == to.0 && node.y == to.1 {
					print!("E")
				} else if node.is_wall {
					print!("O");
				} else if path.contains(node) {
					print!("#");
				} else {
					print!(".");
				}
			}
			println!();
		}
		path
	}
}

fn main() {
	let grid = D2Q9::new(100, 20, |x, y| {
		(x % 10 == 3 && y > 5) || (x % 10 == 8 && y < 15)
		// x / 5 == y && y < 15
	});
	grid.path_and_show((0, 0), (80, 7));
}
