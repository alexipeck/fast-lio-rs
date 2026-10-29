//! ikd-Tree: the incremental k-d tree FAST-LIO keeps its map in
//! (`include/ikd-Tree/ikd_Tree.{h,cpp}`, hku-mars/ikd-Tree, branch `fast_lio`).
//!
//! Same algorithm: lazy deletion labels pushed down on demand, box deletion
//! that marks whole subtrees, insertion with voxel downsampling, and partial
//! rebuilds triggered by the balance (0.6) and deletion (0.5) criteria.
//!
//! Two implementation differences, neither of which changes which points
//! the map holds or what a search returns:
//! - Nodes live in an arena (`Vec`) addressed by index, not raw pointers.
//! - Subtrees of 1500 or more points are rebuilt in place. The C++ hands them
//!   to a background thread and replays operations logged meanwhile.
//!
//! The k-nearest search takes `&self`, so it can run from many threads. Lazy
//! labels only need pushing down inside subtrees already marked fully deleted,
//! and the search never enters those.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::point::Point;

const MINIMAL_UNBALANCED_TREE_SIZE: u32 = 10;
const EPSS: f32 = 1e-6;

/// Axis-aligned box, half-open: `min <= p < max`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxPoint {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

type NodeId = u32;

#[derive(Clone, Debug)]
struct Node {
    point: Point,
    axis: usize,
    tree_size: u32,
    invalid_point_num: u32,
    down_del_num: u32,
    point_deleted: bool,
    tree_deleted: bool,
    point_downsample_deleted: bool,
    tree_downsample_deleted: bool,
    need_push_down_to_left: bool,
    need_push_down_to_right: bool,
    /// Bounds of the subtree's live points, per axis: `[min, max]`.
    range: [[f32; 2]; 3],
    left: Option<NodeId>,
    right: Option<NodeId>,
}

impl Node {
    fn new(point: Point, axis: usize) -> Self {
        Self {
            point,
            axis,
            tree_size: 0,
            invalid_point_num: 0,
            down_del_num: 0,
            point_deleted: false,
            tree_deleted: false,
            point_downsample_deleted: false,
            tree_downsample_deleted: false,
            need_push_down_to_left: false,
            need_push_down_to_right: false,
            range: [[0.0; 2]; 3],
            left: None,
            right: None,
        }
    }
}

fn coord(p: &Point, axis: usize) -> f32 {
    match axis {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    }
}

fn in_box(b: &BoxPoint, p: &Point) -> bool {
    (0..3).all(|a| b.min[a] <= coord(p, a) && b.max[a] > coord(p, a))
}

fn same_point(a: &Point, b: &Point) -> bool {
    (a.x - b.x).abs() < EPSS && (a.y - b.y).abs() < EPSS && (a.z - b.z).abs() < EPSS
}

/// Max-heap entry for the k-nearest search, ordered by squared distance.
#[derive(Clone, Copy)]
struct Candidate {
    dist: f32,
    point: Point,
}

impl PartialEq for Candidate {
    fn eq(&self, o: &Self) -> bool {
        self.dist.total_cmp(&o.dist) == Ordering::Equal
    }
}
impl Eq for Candidate {}
impl PartialOrd for Candidate {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Candidate {
    fn cmp(&self, o: &Self) -> Ordering {
        self.dist.total_cmp(&o.dist)
    }
}

#[derive(Clone, Debug)]
pub struct IkdTree {
    nodes: Vec<Node>,
    free: Vec<NodeId>,
    root: Option<NodeId>,
    delete_criterion_param: f32,
    balance_criterion_param: f32,
    downsample_size: f32,
}

impl Default for IkdTree {
    /// The constructor defaults FAST-LIO uses: delete 0.5, balance 0.6, box 0.2.
    fn default() -> Self {
        Self::new(0.5, 0.6, 0.2)
    }
}

impl IkdTree {
    pub fn new(delete_param: f32, balance_param: f32, box_length: f32) -> Self {
        Self {
            nodes: Vec::new(),
            free: Vec::new(),
            root: None,
            delete_criterion_param: delete_param,
            balance_criterion_param: balance_param,
            downsample_size: box_length,
        }
    }

    pub fn set_downsample_param(&mut self, size: f32) {
        self.downsample_size = size;
    }

    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Nodes in the tree, including deleted ones not yet rebuilt away.
    pub fn size(&self) -> usize {
        self.root
            .map_or(0, |r| self.nodes[r as usize].tree_size as usize)
    }

    /// Points currently in the map.
    pub fn valid_num(&self) -> usize {
        self.root.map_or(0, |r| {
            let n = &self.nodes[r as usize];
            (n.tree_size - n.invalid_point_num) as usize
        })
    }

    /// Replaces the tree with one built from `points`.
    pub fn build(&mut self, mut points: Vec<Point>) {
        self.nodes.clear();
        self.free.clear();
        self.root = self.build_tree(&mut points);
    }

    /// All points in the map, in tree order.
    pub fn flatten(&mut self) -> Vec<Point> {
        let mut out = Vec::new();
        self.flatten_into(self.root, &mut out);
        out
    }

    /// Up to `k` nearest points within `max_dist`, closest first, written into
    /// `points` with their squared distances in `sq_dists`.
    pub fn nearest_search(
        &self,
        point: &Point,
        k: usize,
        max_dist: f32,
        points: &mut Vec<Point>,
        sq_dists: &mut Vec<f32>,
    ) {
        let mut heap = BinaryHeap::with_capacity(2 * k);
        if k > 0 {
            self.search(self.root, k, point, &mut heap, max_dist * max_dist);
        }
        points.clear();
        sq_dists.clear();
        for c in heap.into_sorted_vec() {
            points.push(c.point);
            sq_dists.push(c.dist);
        }
    }

    /// `Add_Points`. With `downsample_on`, each point competes with the map
    /// points in its `downsample_size` voxel, and only the one nearest the voxel
    /// centre is kept. Returns how many downsampled insertions were made.
    pub fn add_points(&mut self, points: &[Point], downsample_on: bool) -> usize {
        let mut added = 0;
        let mut in_voxel = Vec::new();
        for p in points {
            let axis = self.root.map_or(0, |r| self.nodes[r as usize].axis);
            if !downsample_on {
                self.root = Some(self.add_by_point(self.root, *p, true, axis));
                continue;
            }
            let ds = self.downsample_size;
            let mut b = BoxPoint {
                min: [0.0; 3],
                max: [0.0; 3],
            };
            let mut mid = Point::default();
            for a in 0..3 {
                b.min[a] = (coord(p, a) / ds).floor() * ds;
                b.max[a] = b.min[a] + ds;
            }
            mid.x = b.min[0] + (b.max[0] - b.min[0]) / 2.0;
            mid.y = b.min[1] + (b.max[1] - b.min[1]) / 2.0;
            mid.z = b.min[2] + (b.max[2] - b.min[2]) / 2.0;
            in_voxel.clear();
            self.search_by_range(self.root, &b, &mut in_voxel);
            let mut min_dist = p.sq_dist(&mid);
            let mut result = *p;
            for q in &in_voxel {
                let d = q.sq_dist(&mid);
                if d < min_dist {
                    min_dist = d;
                    result = *q;
                }
            }
            if in_voxel.len() > 1 || same_point(p, &result) {
                if !in_voxel.is_empty() {
                    self.root = self.delete_by_range(self.root, &b, true, true).0;
                }
                let axis = self.root.map_or(0, |r| self.nodes[r as usize].axis);
                self.root = Some(self.add_by_point(self.root, result, true, axis));
                added += 1;
            }
        }
        added
    }

    /// `Delete_Point_Boxes`. Returns how many map points were removed.
    pub fn delete_point_boxes(&mut self, boxes: &[BoxPoint]) -> usize {
        let mut deleted = 0;
        for b in boxes {
            let (root, n) = self.delete_by_range(self.root, b, true, false);
            self.root = root;
            deleted += n;
        }
        deleted
    }

    fn alloc(&mut self, node: Node) -> NodeId {
        if let Some(id) = self.free.pop() {
            self.nodes[id as usize] = node;
            id
        } else {
            self.nodes.push(node);
            (self.nodes.len() - 1) as NodeId
        }
    }

    fn release_subtree(&mut self, id: Option<NodeId>) {
        let mut stack: Vec<NodeId> = id.into_iter().collect();
        while let Some(n) = stack.pop() {
            let node = &self.nodes[n as usize];
            stack.extend(node.left);
            stack.extend(node.right);
            self.free.push(n);
        }
    }

    /// `BuildTree`: median split along the axis with the largest extent.
    fn build_tree(&mut self, points: &mut [Point]) -> Option<NodeId> {
        if points.is_empty() {
            return None;
        }
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for p in points.iter() {
            for a in 0..3 {
                min[a] = min[a].min(coord(p, a));
                max[a] = max[a].max(coord(p, a));
            }
        }
        let mut axis = 0;
        for a in 1..3 {
            if max[a] - min[a] > max[axis] - min[axis] {
                axis = a;
            }
        }
        let mid = (points.len() - 1) >> 1;
        points.select_nth_unstable_by(mid, |a, b| coord(a, axis).total_cmp(&coord(b, axis)));
        let id = self.alloc(Node::new(points[mid], axis));
        let (lo, rest) = points.split_at_mut(mid);
        let left = self.build_tree(lo);
        let right = self.build_tree(&mut rest[1..]);
        let node = &mut self.nodes[id as usize];
        node.left = left;
        node.right = right;
        self.update(id);
        Some(id)
    }

    /// `Rebuild`: re-balances a subtree from its live points. Returns the new
    /// subtree root, or `None` if no live points remain.
    fn rebuild(&mut self, id: NodeId) -> Option<NodeId> {
        let mut points = Vec::new();
        self.flatten_into(Some(id), &mut points);
        self.release_subtree(Some(id));
        self.build_tree(&mut points)
    }

    fn criterion_check(&self, id: NodeId) -> bool {
        let n = &self.nodes[id as usize];
        if n.tree_size <= MINIMAL_UNBALANCED_TREE_SIZE {
            return false;
        }
        let son = n
            .left
            .or(n.right)
            .expect("a tree of more than 10 nodes has a child");
        let delete_evaluation = n.invalid_point_num as f32 / n.tree_size as f32;
        let balance_evaluation =
            self.nodes[son as usize].tree_size as f32 / (n.tree_size - 1) as f32;
        delete_evaluation > self.delete_criterion_param
            || balance_evaluation > self.balance_criterion_param
            || balance_evaluation < 1.0 - self.balance_criterion_param
    }

    /// Runs `Criterion_Check` and rebuilds if it fires; returns the subtree root.
    fn maybe_rebuild(&mut self, id: NodeId, allow_rebuild: bool) -> Option<NodeId> {
        if allow_rebuild && self.criterion_check(id) {
            self.rebuild(id)
        } else {
            Some(id)
        }
    }

    fn push_down(&mut self, id: NodeId) {
        let n = self.nodes[id as usize].clone();
        for (needed, child) in [
            (n.need_push_down_to_left, n.left),
            (n.need_push_down_to_right, n.right),
        ] {
            let Some(c) = child.filter(|_| needed) else {
                continue;
            };
            let s = &mut self.nodes[c as usize];
            s.tree_downsample_deleted |= n.tree_downsample_deleted;
            s.point_downsample_deleted |= n.tree_downsample_deleted;
            s.tree_deleted = n.tree_deleted || s.tree_downsample_deleted;
            s.point_deleted = s.tree_deleted || s.point_downsample_deleted;
            if n.tree_downsample_deleted {
                s.down_del_num = s.tree_size;
            }
            s.invalid_point_num = if n.tree_deleted {
                s.tree_size
            } else {
                s.down_del_num
            };
            s.need_push_down_to_left = true;
            s.need_push_down_to_right = true;
        }
        let node = &mut self.nodes[id as usize];
        if node.left.is_some() {
            node.need_push_down_to_left &= !n.need_push_down_to_left;
        }
        if node.right.is_some() {
            node.need_push_down_to_right &= !n.need_push_down_to_right;
        }
    }

    /// `Update`: recomputes counts, deletion labels and live-point bounds from
    /// the children.
    fn update(&mut self, id: NodeId) {
        let n = &self.nodes[id as usize];
        let (point, pd, pdd) = (n.point, n.point_deleted, n.point_downsample_deleted);
        let children: Vec<Node> = [n.left, n.right]
            .into_iter()
            .flatten()
            .map(|c| self.nodes[c as usize].clone())
            .collect();

        let tree_size = 1 + children.iter().map(|c| c.tree_size).sum::<u32>();
        let invalid = u32::from(pd) + children.iter().map(|c| c.invalid_point_num).sum::<u32>();
        let down_del = u32::from(pdd) + children.iter().map(|c| c.down_del_num).sum::<u32>();
        let tree_ds_deleted = pdd && children.iter().all(|c| c.tree_downsample_deleted);
        let tree_deleted = pd && children.iter().all(|c| c.tree_deleted);

        let mut range = [[f32::INFINITY, f32::NEG_INFINITY]; 3];
        let mut include = |r: &[[f32; 2]; 3]| {
            for a in 0..3 {
                range[a][0] = range[a][0].min(r[a][0]);
                range[a][1] = range[a][1].max(r[a][1]);
            }
        };
        let own = [[point.x; 2], [point.y; 2], [point.z; 2]];
        // Everything counts when the whole subtree is deleted or nothing is;
        // otherwise only the live parts do.
        if tree_deleted || (!pd && children.iter().all(|c| !c.tree_deleted)) {
            children.iter().for_each(|c| include(&c.range));
            include(&own);
        } else {
            children
                .iter()
                .filter(|c| !c.tree_deleted)
                .for_each(|c| include(&c.range));
            if !pd {
                include(&own);
            }
        }

        let node = &mut self.nodes[id as usize];
        node.tree_size = tree_size;
        node.invalid_point_num = invalid;
        node.down_del_num = down_del;
        node.tree_downsample_deleted = tree_ds_deleted;
        node.tree_deleted = tree_deleted;
        node.range = range;
    }

    fn flatten_into(&mut self, id: Option<NodeId>, out: &mut Vec<Point>) {
        let Some(id) = id else { return };
        self.push_down(id);
        let n = &self.nodes[id as usize];
        let (left, right) = (n.left, n.right);
        if !n.point_deleted {
            out.push(n.point);
        }
        self.flatten_into(left, out);
        self.flatten_into(right, out);
    }

    /// `Add_by_point`. Returns the root of the (possibly rebuilt) subtree.
    fn add_by_point(
        &mut self,
        id: Option<NodeId>,
        point: Point,
        allow_rebuild: bool,
        father_axis: usize,
    ) -> NodeId {
        let Some(id) = id else {
            let new = self.alloc(Node::new(point, (father_axis + 1) % 3));
            self.update(new);
            return new;
        };
        self.push_down(id);
        let n = &self.nodes[id as usize];
        let axis = n.axis;
        if coord(&point, axis) < coord(&n.point, axis) {
            let child = self.add_by_point(n.left, point, allow_rebuild, axis);
            self.nodes[id as usize].left = Some(child);
        } else {
            let child = self.add_by_point(n.right, point, allow_rebuild, axis);
            self.nodes[id as usize].right = Some(child);
        }
        self.update(id);
        self.maybe_rebuild(id, allow_rebuild)
            .expect("a subtree that just gained a point is not empty")
    }

    /// `Delete_by_range`. Returns the subtree root and the number of live points
    /// deleted.
    fn delete_by_range(
        &mut self,
        id: Option<NodeId>,
        b: &BoxPoint,
        allow_rebuild: bool,
        is_downsample: bool,
    ) -> (Option<NodeId>, usize) {
        let Some(id) = id else { return (None, 0) };
        if self.nodes[id as usize].tree_deleted {
            return (Some(id), 0);
        }
        self.push_down(id);
        let n = &mut self.nodes[id as usize];
        let r = n.range;
        if (0..3).any(|a| b.max[a] <= r[a][0] || b.min[a] > r[a][1]) {
            return (Some(id), 0);
        }
        if (0..3).all(|a| b.min[a] <= r[a][0] && b.max[a] > r[a][1]) {
            n.tree_deleted = true;
            n.point_deleted = true;
            n.need_push_down_to_left = true;
            n.need_push_down_to_right = true;
            let deleted = (n.tree_size - n.invalid_point_num) as usize;
            n.invalid_point_num = n.tree_size;
            if is_downsample {
                n.tree_downsample_deleted = true;
                n.point_downsample_deleted = true;
                n.down_del_num = n.tree_size;
            }
            return (Some(id), deleted);
        }
        let mut deleted = 0;
        if !n.point_deleted && in_box(b, &n.point) {
            n.point_deleted = true;
            deleted += 1;
            if is_downsample {
                n.point_downsample_deleted = true;
            }
        }
        let (left, right) = (n.left, n.right);
        let (left, dl) = self.delete_by_range(left, b, allow_rebuild, is_downsample);
        let (right, dr) = self.delete_by_range(right, b, allow_rebuild, is_downsample);
        let n = &mut self.nodes[id as usize];
        n.left = left;
        n.right = right;
        self.update(id);
        (self.maybe_rebuild(id, allow_rebuild), deleted + dl + dr)
    }

    /// `Search_by_range`: live points inside `b`.
    fn search_by_range(&mut self, id: Option<NodeId>, b: &BoxPoint, out: &mut Vec<Point>) {
        let Some(id) = id else { return };
        self.push_down(id);
        let n = &self.nodes[id as usize];
        let r = n.range;
        if (0..3).any(|a| b.max[a] <= r[a][0] || b.min[a] > r[a][1]) {
            return;
        }
        if (0..3).all(|a| b.min[a] <= r[a][0] && b.max[a] > r[a][1]) {
            self.flatten_into(Some(id), out);
            return;
        }
        if in_box(b, &n.point) && !n.point_deleted {
            out.push(n.point);
        }
        let (left, right) = (n.left, n.right);
        self.search_by_range(left, b, out);
        self.search_by_range(right, b, out);
    }

    /// `calc_box_dist`: squared distance from `p` to a subtree's bounds.
    fn box_dist(&self, id: Option<NodeId>, p: &Point) -> f32 {
        let Some(id) = id else { return f32::INFINITY };
        let mut d = 0.0f32;
        for (a, &[lo, hi]) in self.nodes[id as usize].range.iter().enumerate() {
            let v = coord(p, a);
            if v < lo {
                d += (v - lo) * (v - lo);
            }
            if v > hi {
                d += (v - hi) * (v - hi);
            }
        }
        d
    }

    /// `Search`: branch-and-bound k-nearest, visiting the nearer child first.
    fn search(
        &self,
        id: Option<NodeId>,
        k: usize,
        p: &Point,
        heap: &mut BinaryHeap<Candidate>,
        max_dist_sq: f32,
    ) {
        let Some(id) = id else { return };
        let n = &self.nodes[id as usize];
        if n.tree_deleted || self.box_dist(Some(id), p) > max_dist_sq {
            return;
        }
        if !n.point_deleted {
            let dist = p.sq_dist(&n.point);
            if dist <= max_dist_sq
                && (heap.len() < k || dist < heap.peek().map_or(f32::INFINITY, |c| c.dist))
            {
                if heap.len() >= k {
                    heap.pop();
                }
                heap.push(Candidate {
                    dist,
                    point: n.point,
                });
            }
        }
        let top = |heap: &BinaryHeap<Candidate>| heap.peek().map_or(f32::INFINITY, |c| c.dist);
        let dl = self.box_dist(n.left, p);
        let dr = self.box_dist(n.right, p);
        if heap.len() < k || (dl < top(heap) && dr < top(heap)) {
            let (first, d_second, second) = if dl <= dr {
                (n.left, dr, n.right)
            } else {
                (n.right, dl, n.left)
            };
            self.search(first, k, p, heap, max_dist_sq);
            if heap.len() < k || d_second < top(heap) {
                self.search(second, k, p, heap, max_dist_sq);
            }
        } else {
            if dl < top(heap) {
                self.search(n.left, k, p, heap, max_dist_sq);
            }
            if dr < top(heap) {
                self.search(n.right, k, p, heap, max_dist_sq);
            }
        }
    }
}
