const std = @import("std");
const calc = @import("calc.zig");
const data = @import("data.zig");
const index = @import("index.zig");
const svg = @import("svg.zig");
const vol = @import("volume.zig");
const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const OrientedBox2f = vol.OrientedBox2f;

/// A data structure that covers a square region (size * size) of 2D Euclidean space.
pub fn SquareTree(
    comptime Indexer: type, // Indexer used to structure tree.
    comptime Volume: type, // Type of volumes stored in leaf nodes.
    comptime ClientId: type, // Caller-chosen ID type.
) type {
    return struct {
        indexer: Indexer,
        num_volumes: usize = 0, // number of volumes currently stored
        max_half_extent: Vec2f = calc.zero2f, // largest half-extent of any stored volume
        bounds_valid: bool = false, // false if bounds need to be updated
        node_bvs: [depth][]Box2f, // BVs for all nodes
        leaf_data: []Volume, // all volumes, sorted by leaf number
        leaf_ids: []ClientId, // client ids for leaf_data, in the same order
        staged_data: []Volume, // volumes in insertion order, unsorted
        staged_ids: []ClientId, // client ids for staged_data, in the same order
        leaf_starts: []StartIndex, // stores the leaf_data start index for each leaf
        leaf_counts: []DataIndex, // the number of volumes within each leaf node
        bfs_buff_a: []CurveIndex, // Scratch buffer for findOverlapsBfs
        bfs_buff_b: []CurveIndex, // Scratch buffer for findOverlapsBfs
        top_occupied: data.BoundedList(CurveIndex), // indexes of non-empty nodes on level 0

        pub const depth = Indexer.depth;
        pub const nodes_in_level = Indexer.nodes_in_level;
        pub const num_leaves = Indexer.num_leaves;
        pub const OverlapPair = [2]ClientId;
        pub const Neighbour = struct { id: ClientId, dist: f32 };
        pub const VolumeType = Volume;
        pub const ClientIdType = ClientId;
        pub const compressed = Indexer.top_levels > 1;
        const CurveIndex = Indexer.CurveIndex;
        const DataIndex = u16; // Used to index volumes within leaf nodes.
        const StartIndex = u32; // Offset into leaf_data/leaf_ids
        const Self = @This();

        pub fn init(
            allocator: std.mem.Allocator,
            bound_1: Vec2f, // a corner of the space to be covered
            bound_2: Vec2f, // the opposite corner of the space
            capacity: u32, // bounds heap-allocated memory
        ) !Self {
            const indexer = try Indexer.init(bound_1, bound_2);
            const leaf_data = try allocator.alloc(Volume, capacity);
            errdefer allocator.free(leaf_data);
            const leaf_ids = try allocator.alloc(ClientId, capacity);
            errdefer allocator.free(leaf_ids);
            const leaf_starts = try allocator.alloc(StartIndex, num_leaves);
            errdefer allocator.free(leaf_starts);
            @memset(leaf_starts, 0);
            const leaf_counts = try allocator.alloc(DataIndex, num_leaves);
            errdefer allocator.free(leaf_counts);
            @memset(leaf_counts, 0);

            const staged_data = try allocator.alloc(Volume, capacity);
            errdefer allocator.free(staged_data);
            const staged_ids = try allocator.alloc(ClientId, capacity);
            errdefer allocator.free(staged_ids);

            const bfs_buff_a = try allocator.alloc(CurveIndex, num_leaves);
            errdefer allocator.free(bfs_buff_a);
            const bfs_buff_b = try allocator.alloc(CurveIndex, num_leaves);
            errdefer allocator.free(bfs_buff_b);

            const top_occupied = try allocator.alloc(CurveIndex, nodes_in_level[0]);
            errdefer allocator.free(top_occupied);

            var node_bvs: [depth][]Box2f = undefined;
            var levels_allocated: usize = 0;
            errdefer for (node_bvs[0..levels_allocated]) |s| allocator.free(s);
            inline for (0..depth) |lvl| {
                node_bvs[lvl] = try allocator.alloc(Box2f, nodes_in_level[lvl]);
                levels_allocated += 1;
            }

            return Self{
                .indexer = indexer,
                .node_bvs = node_bvs,
                .leaf_data = leaf_data,
                .leaf_ids = leaf_ids,
                .leaf_starts = leaf_starts,
                .leaf_counts = leaf_counts,
                .staged_data = staged_data,
                .staged_ids = staged_ids,
                .bfs_buff_a = bfs_buff_a,
                .bfs_buff_b = bfs_buff_b,
                .top_occupied = data.BoundedList(CurveIndex).init(top_occupied),
            };
        }

        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            for (self.node_bvs) |v| allocator.free(v);
            allocator.free(self.top_occupied.items);
            allocator.free(self.bfs_buff_b);
            allocator.free(self.bfs_buff_a);
            allocator.free(self.staged_ids);
            allocator.free(self.staged_data);
            allocator.free(self.leaf_counts);
            allocator.free(self.leaf_starts);
            allocator.free(self.leaf_ids);
            allocator.free(self.leaf_data);
        }

        /// Adds volumes to the grid and stores their associated client ids (order must match).
        /// The volumes are staged: call `updateBounds` before querying.
        pub fn addVolumes(self: *Self, vols: []const Volume, client_ids: []const ClientId) !void {
            if (self.num_volumes + vols.len > self.staged_data.len) return error.CapacityExceeded;
            if (vols.len != client_ids.len) return error.VolumeIndexLengthMismatch;
            for (vols, client_ids) |v, c| {
                const leaf_num = self.indexer.getLeafIndexForPoint(v.getCentre());
                const data_index = self.leaf_counts[leaf_num];
                if (data_index == std.math.maxInt(DataIndex)) return error.DataIndexRangeExceeded;

                if (compressed) {
                    const bb = v.getBoundingBox();
                    const he = calc.scaledVec(0.5, bb.max - bb.min);
                    self.max_half_extent = @max(self.max_half_extent, he);
                }
                self.staged_data[self.num_volumes] = v;
                self.staged_ids[self.num_volumes] = c;
                self.leaf_counts[leaf_num] = data_index + 1;
                self.num_volumes += 1;
            }
            self.bounds_valid = false;
        }

        /// Removes all volumes stored in leaf-nodes of the grid.
        pub fn clearStoredVolumes(self: *Self) void {
            @memset(self.leaf_counts, 0);
            self.bounds_valid = false;
            self.max_half_extent = calc.zero2f;
            self.num_volumes = 0;
            self.top_occupied.clear();
        }

        /// Diagnostic: the largest number of volumes staged in any single leaf.
        pub fn maxLeafOccupancy(self: *const Self) DataIndex {
            var max_count: DataIndex = 0;
            for (self.leaf_counts) |count| max_count = @max(max_count, count);
            return max_count;
        }

        /// Relocates the tree to a new position.
        /// Tree must be empty: call `clearStoredVolumes` first.
        pub fn relocate(self: *Self, new_min: Vec2f, new_max: Vec2f) !void {
            if (self.num_volumes > 0) return error.CannotRelocateOccupiedTree;
            self.indexer = try Indexer.init(new_min, new_max);
        }

        /// Grows all bounding volumes to cover all points within them.
        /// Sorts any volumes staged by `addVolume` into their final position.
        pub fn updateBounds(self: *Self) !void {
            self.sortStagedVolumes();
            // start with leaf nodes first
            for (self.node_bvs[depth - 1], 0..) |*bv, i| {
                var box = vol.empty_box;
                for (self.getLeafVolumes(@intCast(i))) |other_vol| {
                    box = vol.getEncompassingBox(box, other_vol);
                }
                bv.* = box;
                if (depth == 1 and !box.isEmpty()) {
                    try self.top_occupied.add(@intCast(i));
                }
            }
            // parent nodes, working up from the level above the leaves
            for (2..depth + 1) |i| {
                const lvl = depth - i;
                const child_bvs = self.node_bvs[lvl + 1];
                for (self.node_bvs[lvl], 0..) |*bv, j| {
                    var box = vol.empty_box;
                    const first_child = Indexer.getFirstChild(@truncate(j));
                    for (child_bvs[first_child..][0..Indexer.num_children]) |c| {
                        box = vol.getEncompassingBox(box, c);
                    }
                    bv.* = box;
                    if (lvl == 0 and !box.isEmpty()) {
                        try self.top_occupied.add(@intCast(j));
                    }
                }
            }
            self.bounds_valid = true;
        }

        /// Returns client ids of all stored volumes that overlap the query volume.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        pub fn findOverlaps(self: *const Self, res_buff: []ClientId, query_vol: anytype) ![]ClientId {
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            return try self.findOverlapsBfs(res_buff, query_vol);
        }

        /// Finds every pair of stored volumes that overlap each other.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        pub fn findSelfOverlaps(self: *const Self, res_buff: []OverlapPair) ![]OverlapPair {
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            const top_bvs = self.node_bvs[0];
            const occupied = self.top_occupied.getItems();
            var res_list = data.BoundedList(OverlapPair).init(res_buff);
            if (compressed) { // check the nearby level 0 nodes only
                for (occupied) |a| {
                    const bv_a = top_bvs[a];
                    const neighbourhood: Box2f = .{
                        .min = bv_a.min - self.max_half_extent,
                        .max = bv_a.max + self.max_half_extent,
                    };
                    const near = self.indexer.getTopLevelIndexesForBox(self.bfs_buff_a, neighbourhood);
                    for (near) |b| {
                        if (b < a) continue; // the pair is visited from the lower node
                        try self.findSelfOverlapsDtt(0, &res_list, a, bv_a, b, top_bvs[b]);
                    }
                }
            } else { // check the occupied level 0 nodes from each one onwards
                for (occupied, 0..) |a, i| {
                    const bv_a = top_bvs[a];
                    for (occupied[i..]) |b| {
                        try self.findSelfOverlapsDtt(0, &res_list, a, bv_a, b, top_bvs[b]);
                    }
                }
            }
            return res_list.getItems();
        }

        /// Finds stored volumes nearest to the query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        pub fn findNearestNeighbours(
            self: *const Self,
            buf: []Neighbour,
            point: Vec2f,
            k: u16,
            max_dist: f32,
            exclude_id: ?ClientId,
        ) ![]Neighbour {
            if (k > buf.len) return error.UndersizedBuffer;
            const leaf_index = self.indexer.getLeafIndexForPoint(point);
            var len: usize = 0;
            var iter: u16 = 0;
            var furthest_dist: f32 = 0;
            var next_min_dist: f32 = 0;
            while (next_min_dist < max_dist and (len < k or next_min_dist < furthest_dist)) {
                const leaves = try Indexer.getLeafCellNeighbours(self.bfs_buff_a, leaf_index, iter);
                for (leaves) |i| {
                    const vols = self.getLeafVolumes(i);
                    for (vols, self.getLeafIds(i)) |v, client_id| {
                        if (exclude_id != null and exclude_id.? == client_id) continue;
                        const dist_squared = calc.squaredSum(v.getCentre() - point);
                        const threshold = if (len < k) max_dist else furthest_dist;
                        if (dist_squared >= threshold * threshold) continue;
                        const dist = @sqrt(dist_squared);
                        const n_info = Neighbour{ .id = client_id, .dist = dist };
                        var pos = if (len < k) len else k - 1;
                        while (pos > 0 and buf[pos - 1].dist > dist) : (pos -= 1) {
                            buf[pos] = buf[pos - 1];
                        }
                        buf[pos] = n_info;
                        if (len < k) len += 1;
                        furthest_dist = buf[len - 1].dist;
                    }
                }
                next_min_dist = calc.asf32(iter) * self.indexer.cell_size;
                iter += 1;
            }
            return buf[0..len];
        }

        /// Draws a tree's grid subdivisions, cell labels, and stored volumes to an svg file.
        /// Accepts a pointer to any tree exposing the same public interface as `SquareTree`.
        pub fn drawTreeSvg(
            self: *const Self,
            allocator: std.mem.Allocator,
            show_client_ids: bool,
        ) !svg.Canvas {
            const bgs: svg.ShapeStyle = .{
                .fill_active = true,
                .fill_hsl = .{ 0, 0, 95 },
                .stroke_active = false,
            };
            var canvas = try svg.Canvas.init(allocator, self.indexer.min_pt, self.indexer.max_pt, bgs);
            errdefer canvas.deinit(allocator);
            const extent = self.indexer.max_pt - self.indexer.min_pt;
            const scale = @reduce(.Max, extent) / 1024.0;

            // draw grid subdivisions + cell labels, finest level first
            var palette = try svg.RandomHslPalette.init(allocator, depth, 0);
            defer palette.deinit(allocator);
            var label_buff: [16]u8 = undefined;
            for (calc.getReversedRange(Indexer.LevelIndex, depth)) |lvl| {
                const style: svg.ShapeStyle = .{
                    .stroke_hsl = palette.hsl_colours[lvl],
                    .stroke_width = scale * calc.asf32(depth - lvl),
                };

                const font_size: f32 = scale * (6.0 + 4.0 * calc.asf32(depth - lvl));
                for (0..nodes_in_level[lvl]) |i| {
                    const node_index: CurveIndex = @intCast(i);
                    const cell = self.indexer.getCellBoundaryAtLevel(lvl, node_index);
                    const label_width = (std.math.log2_int(usize, nodes_in_level[lvl]) + 3) / 4;
                    const label = try std.fmt.bufPrint(&label_buff, "{X:0>[1]}", .{ node_index, label_width });
                    try canvas.addRectangle(allocator, cell.min, cell.max, style);
                    try canvas.addText(allocator, cell.getCentre(), label, font_size, style.stroke_hsl);
                }
            }

            // find every client id that participates in an overlap
            const volumes = self.leaf_data[0..self.num_volumes];
            const ids = self.leaf_ids[0..self.num_volumes];
            const overlap_buff = try allocator.alloc(OverlapPair, 16 * volumes.len);
            defer allocator.free(overlap_buff);
            const pairs = try self.findSelfOverlaps(overlap_buff);
            var overlapping = std.AutoHashMap(ClientId, void).init(allocator);
            defer overlapping.deinit();
            for (pairs) |pair| {
                try overlapping.put(pair[0], {});
                try overlapping.put(pair[1], {});
            }

            // draw the stored volumes, colouring overlapping ones differently
            const default_style: svg.ShapeStyle = .{ .stroke_hsl = .{ 0, 0, 20 }, .stroke_width = scale * 1.0 };
            const overlap_style: svg.ShapeStyle = .{ .stroke_hsl = .{ 0, 80, 45 }, .stroke_width = scale * 2.0 };
            const id_label_hsl: [3]u9 = .{ 0, 0, 0 };
            const id_label_font_size: f32 = scale * 10.0;
            var id_buff: [20]u8 = undefined;
            for (volumes, ids) |v, id| {
                const style = if (overlapping.contains(id)) overlap_style else default_style;
                if (Volume == Ball2f) {
                    try canvas.addCircle(allocator, v.centre, v.radius, style);
                } else if (Volume == Box2f) {
                    try canvas.addRectangle(allocator, v.min, v.max, style);
                } else if (Volume == OrientedBox2f) {
                    var corners = v.getCorners();
                    try canvas.addPolygon(allocator, &corners, style);
                } else {
                    @compileError("drawTreeSvg: unsupported volume type " ++ @typeName(Volume));
                }
                if (show_client_ids) {
                    const label = try std.fmt.bufPrint(&id_buff, "{d}", .{id});
                    try canvas.addText(allocator, v.getCentre(), label, id_label_font_size, id_label_hsl);
                }
            }

            return canvas;
        }

        /// Gets the volumes stored in the specified leaf node, in insertion order.
        fn getLeafVolumes(self: *const Self, leaf_num: CurveIndex) []const Volume { // TODO: check perf before/after inlining
            const start = self.leaf_starts[leaf_num];
            return self.leaf_data[start..][0..self.leaf_counts[leaf_num]];
        }

        /// Gets the client ids stored in the specified leaf node, in the same order as getLeafVolumes.
        fn getLeafIds(self: *const Self, leaf_num: CurveIndex) []const ClientId { // TODO: check perf before/after inlining
            const start = self.leaf_starts[leaf_num];
            return self.leaf_ids[start..][0..self.leaf_counts[leaf_num]];
        }

        /// Sorts the staged volumes into leaf_data by leaf number and fills leaf_starts.
        fn sortStagedVolumes(self: *Self) void {
            // compute each leaf's start offset
            var offset: StartIndex = 0;
            for (self.leaf_starts, self.leaf_counts) |*start, count| {
                start.* = offset;
                offset += count;
            }
            // scatter, using leaf_starts as the per-leaf write cursor
            const num_vols = self.num_volumes;
            for (self.staged_data[0..num_vols], self.staged_ids[0..num_vols]) |v, id| {
                const leaf_num = self.indexer.getLeafIndexForPoint(v.getCentre());
                const write_index = self.leaf_starts[leaf_num];
                self.leaf_data[write_index] = v;
                self.leaf_ids[write_index] = id;
                self.leaf_starts[leaf_num] += 1;
            }
            // rewind the cursors, recovering the start offsets
            for (self.leaf_starts, self.leaf_counts) |*start, count| {
                start.* -= count;
            }
        }

        /// Performs a BFS for stored volumes that overlap with the provided query volume.
        fn findOverlapsBfs(self: *const Self, res_buff: []ClientId, query_vol: anytype) ![]ClientId {
            // search through higher-level nodes first
            const query_aabb: Box2f = query_vol.getBoundingBox();
            const buff_1 = self.bfs_buff_a;
            const buff_2 = self.bfs_buff_b;
            var search_slice: []CurveIndex = undefined;
            if (compressed) { // check the neighbouring level 0 nodes only
                const neighbourhood: Box2f = .{
                    .min = query_aabb.min - self.max_half_extent,
                    .max = query_aabb.max + self.max_half_extent,
                };
                search_slice = self.indexer.getTopLevelIndexesForBox(buff_1, neighbourhood);
            } else { // check all level 0 nodes
                for (0..nodes_in_level[0]) |k| buff_1[k] = @intCast(k);
                search_slice = buff_1[0..nodes_in_level[0]];
            }
            var next_slice: []CurveIndex = buff_2;
            for (0..depth - 1) |lvl| {
                var next_offset: usize = 0;
                for (search_slice) |i| {
                    const node_vol = self.node_bvs[lvl][i];
                    if (!vol.checkVolumesOverlap(query_aabb, node_vol)) continue;
                    const first_child = Indexer.getFirstChild(i);
                    for (0..Indexer.num_children) |k| {
                        next_slice[next_offset + k] = first_child + @as(CurveIndex, @intCast(k));
                    }
                    next_offset += Indexer.num_children;
                }
                // Swap the buffers
                const filled = next_slice[0..next_offset];
                next_slice = search_slice.ptr[0..num_leaves];
                search_slice = filled;
            }
            // check the surviving leaf nodes for overlaps
            var res_len: usize = 0;
            for (search_slice) |i| {
                const leaf_vol = self.node_bvs[depth - 1][i];
                if (!vol.checkVolumesOverlap(query_aabb, leaf_vol)) continue;
                const items = self.getLeafVolumes(i);
                const ids = self.getLeafIds(i);
                for (items, ids) |stored_vol, id| {
                    if (!vol.checkVolumesOverlap(query_vol, stored_vol)) continue;
                    if (res_buff.len == res_len) return error.OverlapBufferCapacityExceeded;
                    res_buff[res_len] = id;
                    res_len += 1;
                }
            }
            return res_buff[0..res_len];
        }

        // TODO: provide public version of dual tree traversal for separate tree-vs-tree checks:
        // reference algorithm: https://arxiv.org/pdf/2012.05348

        fn findSelfOverlapsDtt(
            self: *const Self,
            comptime lvl: u4,
            res_list: *data.BoundedList(OverlapPair),
            idx_a: CurveIndex,
            box_a: Box2f,
            idx_b: CurveIndex,
            box_b: Box2f,
        ) !void {
            std.debug.assert(idx_a <= idx_b);
            if (!vol.checkVolumesOverlap(box_a, box_b)) return;
            if (comptime lvl == depth - 1) {
                return self.findLeafPairOverlaps(res_list, idx_a, idx_b, box_b);
            }
            const same_node = idx_a == idx_b;
            var buff_a: [Indexer.num_children]CurveIndex = undefined;
            const live_a = self.findOverlappingChildren(lvl, &buff_a, idx_a, box_b);
            if (live_a.len == 0) return;
            var buff_b: [Indexer.num_children]CurveIndex = undefined;
            const live_b = if (same_node) live_a else self.findOverlappingChildren(lvl, &buff_b, idx_b, box_a);
            const child_bvs = self.node_bvs[lvl + 1];
            for (live_a, 0..) |child_a, i| {
                const box_ca = child_bvs[child_a];
                const first_b = if (same_node) i else 0;
                for (live_b[first_b..]) |child_b| {
                    try self.findSelfOverlapsDtt(
                        lvl + 1,
                        res_list,
                        child_a,
                        box_ca,
                        child_b,
                        child_bvs[child_b],
                    );
                }
            }
        }

        fn findLeafPairOverlaps(
            self: *const Self,
            res_list: *data.BoundedList(OverlapPair),
            idx_a: CurveIndex,
            idx_b: CurveIndex,
            box_b: Box2f,
        ) !void {
            const vols_a = self.getLeafVolumes(idx_a);
            const ids_a = self.getLeafIds(idx_a);
            const vols_b = self.getLeafVolumes(idx_b);
            const ids_b = self.getLeafIds(idx_b);
            const same_leaf = idx_a == idx_b;
            for (vols_a, ids_a, 0..) |vol_a, id_a, i| {
                if (!same_leaf and !vol.checkVolumesOverlap(vol_a.getBoundingBox(), box_b)) continue;
                const first_b = if (same_leaf) i + 1 else 0;
                for (vols_b[first_b..], ids_b[first_b..]) |vol_b, id_b| {
                    if (!vol.checkVolumesOverlap(vol_a, vol_b)) continue;
                    res_list.add(.{ id_a, id_b }) catch return error.OverlapBufferCapacityExceeded;
                }
            }
        }

        fn findOverlappingChildren(
            self: *const Self,
            comptime lvl: u4,
            buff: *[Indexer.num_children]CurveIndex,
            node_index: CurveIndex,
            bounds: Box2f,
        ) []CurveIndex {
            const first_child = Indexer.getFirstChild(node_index);
            const child_bvs = self.node_bvs[lvl + 1];
            var len: usize = 0;
            for (0..Indexer.num_children) |k| {
                const child = first_child + @as(CurveIndex, @intCast(k));
                if (!vol.checkVolumesOverlap(child_bvs[child], bounds)) continue;
                buff[len] = child;
                len += 1;
            }
            return buff[0..len];
        }
    };
}

const testing = std.testing;
const test_alloc = testing.allocator;
const test_capacity = 1000;

test "square tree init" {
    const IndexerM2x8 = index.Indexer2f(2, 1, 7, .Morton);
    const Tree2x8 = SquareTree(IndexerM2x8, Ball2f, u32);
    var qt = try Tree2x8.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer qt.deinit(test_alloc);

    const IndexerSwizz4x6 = index.Indexer2f(4, 1, 5, .Zigzag);
    const Tree4x4 = SquareTree(IndexerSwizz4x6, Ball2f, u32);
    var ht = try Tree4x4.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer ht.deinit(test_alloc);
}

test "hex tree overlap ball" {
    const IndexerSwizz4x2 = index.Indexer2f(4, 1, 1, .Zigzag);
    const HexTree2 = SquareTree(IndexerSwizz4x2, Ball2f, u32);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer tree.deinit(test_alloc);

    var balls = [3]Ball2f{
        .{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 },
        .{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 },
        .{ .centre = Vec2f{ 0.2, 0.7 }, .radius = 0.1 },
    };
    const indexes = calc.getRange(u32, balls.len);
    try tree.addVolumes(balls[0..], &indexes);
    try tree.updateBounds();

    const leaf_a = tree.indexer.getLeafIndexForPoint(balls[0].getCentre());
    const leaf_b = tree.indexer.getLeafIndexForPoint(balls[1].getCentre());
    const leaf_c = tree.indexer.getLeafIndexForPoint(balls[2].getCentre());
    try testing.expectEqual(false, leaf_a == leaf_b);
    try testing.expectEqual(false, leaf_a == leaf_c);
    try testing.expectEqual(false, leaf_b == leaf_c);

    var id_buff: [8]u32 = undefined;
    const query_region_1 = Ball2f{ .centre = Vec2f{ 0.9, 0.5 }, .radius = 0.1 };
    const overlap_ids_1 = try tree.findOverlaps(&id_buff, query_region_1);
    try testing.expectEqual(0, overlap_ids_1.len);

    const query_region_2 = Ball2f{ .centre = Vec2f{ -3.5, -3.5 }, .radius = 1.0 };
    const overlap_ids_2 = try tree.findOverlaps(&id_buff, query_region_2);
    try testing.expectEqual(0, overlap_ids_2.len);

    const query_region_3 = Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 };
    const overlap_ids_3 = try tree.findOverlaps(&id_buff, query_region_3);
    try testing.expectEqual(3, overlap_ids_3.len);

    // a overlaps b, and b overlaps c, but a does not overlap c
    var pair_buff: [8]HexTree2.OverlapPair = undefined;
    const pairs = try tree.findSelfOverlaps(&pair_buff);
    try testing.expectEqual(2, pairs.len);
}

test "hex tree overlap box" {
    const Indexer = index.Indexer2f(4, 1, 1, .Zigzag);
    const HexTree2 = SquareTree(Indexer, Box2f, u32);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer tree.deinit(test_alloc);

    const boxes = [_]Box2f{
        .{ .min = .{ -0.2, -0.4 }, .max = .{ 0.6, 0.4 } },
        .{ .min = .{ 0.0, 0.3 }, .max = .{ 0.4, 0.7 } },
        .{ .min = .{ 0.1, 0.6 }, .max = .{ 0.3, 0.8 } },
    };
    const indexes = calc.getRange(u32, boxes.len);
    try tree.addVolumes(&boxes, &indexes);

    try tree.updateBounds();

    const leaf_0 = tree.indexer.getLeafIndexForPoint(boxes[0].getCentre());
    const leaf_1 = tree.indexer.getLeafIndexForPoint(boxes[1].getCentre());
    const leaf_2 = tree.indexer.getLeafIndexForPoint(boxes[2].getCentre());
    try testing.expectEqual(false, leaf_0 == leaf_1);
    try testing.expectEqual(false, leaf_0 == leaf_2);
    try testing.expectEqual(false, leaf_1 == leaf_2);

    var id_buff: [8]u32 = undefined;
    const query_region_1 = Box2f{ .min = [2]f32{ 0.8, 0.4 }, .max = [2]f32{ 1.0, 0.6 } };
    const overlap_ids_1 = try tree.findOverlaps(&id_buff, query_region_1);
    try testing.expectEqual(0, overlap_ids_1.len);

    const query_region_2 = Box2f{ .min = @splat(-4.5), .max = @splat(-2.5) };
    const overlap_ids_2 = try tree.findOverlaps(&id_buff, query_region_2);
    try testing.expectEqual(0, overlap_ids_2.len);

    const query_region_3 = Box2f{ .min = [2]f32{ 0.0, 0.3 }, .max = [2]f32{ 0.4, 0.7 } };
    const overlap_3 = try tree.findOverlaps(&id_buff, query_region_3);
    try testing.expectEqual(3, overlap_3.len);

    // a overlaps b, and b overlaps c, but a does not overlap c
    var pair_buff: [8]HexTree2.OverlapPair = undefined;
    const pairs = try tree.findSelfOverlaps(&pair_buff);
    try testing.expectEqual(2, pairs.len);
}

test "square tree add remove" {
    const IndexerM2x4 = index.Indexer2f(4, 1, 1, .Morton);
    const QuadTree = SquareTree(IndexerM2x4, Ball2f, u32);
    var qt = try QuadTree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer qt.deinit(test_alloc);

    const test_bodies = [_]Ball2f{
        Ball2f{ .centre = Vec2f{ 0.2, 0.0 }, .radius = 0.4 },
        Ball2f{ .centre = Vec2f{ 0.2, 0.5 }, .radius = 0.2 },
        Ball2f{ .centre = Vec2f{ 0.2, 0.9 }, .radius = 0.1 },
    };
    const indexes = calc.getRange(u32, test_bodies.len);
    try qt.addVolumes(&test_bodies, &indexes);
    try testing.expectEqual(3, qt.num_volumes);
    try qt.updateBounds();

    for (0..QuadTree.num_leaves) |leaf_num_usize| {
        const leaf_num: QuadTree.CurveIndex = @intCast(leaf_num_usize);
        for (qt.getLeafVolumes(leaf_num), qt.getLeafIds(leaf_num)) |v, id| {
            try testing.expectEqual(test_bodies[id].centre, v.centre);
            try testing.expectEqual(test_bodies[id].radius, v.radius);
        }
    }

    qt.clearStoredVolumes();
    try testing.expectEqual(0, qt.num_volumes);
}

test "staged volumes keep their insertion rank within a leaf" {
    const IndexerM4x2 = index.Indexer2f(4, 1, 1, .Morton);
    const QuadTree = SquareTree(IndexerM4x2, Ball2f, u32);
    var qt = try QuadTree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity);
    defer qt.deinit(test_alloc);

    // interleave insertions between two distant leaves, so the scatter has to reorder.
    var radii: [6]f32 = undefined;
    for (0..6) |i| {
        radii[i] = 0.01 * @as(f32, @floatFromInt(i + 1));
        const centre = if (i % 2 == 0) Vec2f{ 0.1, 0.1 } else Vec2f{ 0.9, 0.9 };
        const balls = [_]Ball2f{.{ .centre = centre, .radius = radii[i] }};
        const indexes = [_]u32{@intCast(i)};
        try qt.addVolumes(&balls, &indexes);
    }
    try qt.updateBounds();

    // ranks are assigned per leaf, so both leaves should see their volumes in insertion order
    const leaf_even = qt.indexer.getLeafIndexForPoint(Vec2f{ 0.1, 0.1 });
    const leaf_odd = qt.indexer.getLeafIndexForPoint(Vec2f{ 0.9, 0.9 });
    const even_vols = qt.getLeafVolumes(leaf_even);
    const odd_vols = qt.getLeafVolumes(leaf_odd);
    try testing.expectEqual(3, even_vols.len);
    try testing.expectEqual(3, odd_vols.len);
    for (even_vols, 0..) |v, rank| try testing.expectEqual(radii[rank * 2], v.radius);
    for (odd_vols, 0..) |v, rank| try testing.expectEqual(radii[rank * 2 + 1], v.radius);
}

test "findSelfOverlaps agrees with brute force" {
    // Compare the tree's answer against simple pairwise approach
    const Trees = .{
        SquareTree(index.Indexer2f(2, 1, 3, .U), Ball2f, u32),
        SquareTree(index.Indexer2f(4, 1, 2, .Zigzag), Ball2f, u32),
        SquareTree(index.Indexer2f(4, 1, 1, .Morton), Box2f, u32),
        SquareTree(index.Indexer2f(2, 1, 4, .Morton), OrientedBox2f, u32),
        SquareTree(index.Indexer2f(4, 2, 1, .Morton), Ball2f, u32),
        SquareTree(index.Indexer2f(4, 2, 1, .Zigzag), OrientedBox2f, u32),
        SquareTree(index.Indexer2f(2, 3, 2, .Morton), Box2f, u32),
        SquareTree(index.Indexer2f(4, 3, 0, .Morton), Ball2f, u32),
        SquareTree(index.Indexer2f(2, 1, 5, .Morton), Ball2f, u32),
        SquareTree(index.Indexer2f(2, 6, 0, .Morton), Box2f, u32),
    };
    const num_vols = 300;
    var prng = std.Random.DefaultPrng.init(7);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);

    inline for (Trees) |Tree| {
        const bodies = test_vols.getRandomBodies(Tree.VolumeType);
        var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols);
        defer tree.deinit(test_alloc);
        const indexes = calc.getRange(u32, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBounds();
        // brute force reference
        var expected: std.ArrayList(Tree.OverlapPair) = .empty;
        defer expected.deinit(test_alloc);
        for (bodies, 0..) |a, i| {
            for (bodies[i + 1 ..], i + 1..) |b, j| {
                if (vol.checkVolumesOverlap(a, b)) {
                    try expected.append(test_alloc, .{ @intCast(i), @intCast(j) });
                }
            }
        }

        const pair_buff = try test_alloc.alloc(Tree.OverlapPair, num_vols * num_vols);
        defer test_alloc.free(pair_buff);
        const actual = try tree.findSelfOverlaps(pair_buff);

        try testing.expect(expected.items.len > 0); // the test must not pass vacuously
        calc.sortPairsLessThan(Tree.ClientIdType, expected.items);
        calc.sortPairsLessThan(Tree.ClientIdType, actual);
        try testing.expectEqualSlices(Tree.OverlapPair, expected.items, actual);
    }
}

test "findOverlaps agrees with brute force" {
    //
    const Trees = .{
        SquareTree(index.Indexer2f(4, 1, 2, .Morton), Ball2f, u32),
        SquareTree(index.Indexer2f(2, 2, 2, .Morton), Box2f, u32),
        SquareTree(index.Indexer2f(4, 3, 1, .Zigzag), Ball2f, u32),
    };
    const num_vols = 500;
    var prng = std.Random.DefaultPrng.init(42);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);

    inline for (Trees) |Tree| {
        const bodies = test_vols.getRandomBodies(Tree.VolumeType);
        var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols);
        defer tree.deinit(test_alloc);
        const indexes = calc.getRange(u32, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBounds();

        const id_buff = try test_alloc.alloc(Tree.ClientIdType, num_vols);
        defer test_alloc.free(id_buff);
        var total: usize = 0;
        for (bodies) |query| {
            var expected: usize = 0;
            for (bodies) |other| {
                if (vol.checkVolumesOverlap(query, other)) expected += 1;
            }
            const found = try tree.findOverlaps(id_buff, query);
            try testing.expectEqual(expected, found.len);
            total += found.len;
        }
        try testing.expect(total > num_vols);
    }

    // TODO: add brute-force test for find-nearest neighbours
}

test "draw square trees svg" {
    const IndexerM4x2 = index.Indexer2f(4, 1, 1, .Morton);
    const Trees = .{
        SquareTree(IndexerM4x2, Ball2f, u32),
        SquareTree(IndexerM4x2, Box2f, u32),
        SquareTree(IndexerM4x2, OrientedBox2f, u32),
    };
    const min_pt = Vec2f{ 0, 0 };
    const max_pt = Vec2f{ 1024, 1024 };
    const num_vols = 50;

    // moderately-sized volumes clustered around the canvas centre, so overlaps are near-certain.
    var prng = std.Random.DefaultPrng.init(0);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 20, .max = 60 } },
        .{ .normal = .{ .mean = 512, .stddev = 250 } },
    );
    defer test_vols.deinit(test_alloc);

    inline for (Trees) |Tree| {
        var tree = try Tree.init(test_alloc, min_pt, max_pt, num_vols);
        defer tree.deinit(test_alloc);
        const bodies = test_vols.getRandomBodies(Tree.VolumeType);
        const indexes = calc.getRange(u32, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBounds();

        var canvas = try tree.drawTreeSvg(test_alloc, true);
        defer canvas.deinit(test_alloc);

        var buff: [512]u8 = undefined;
        const fpath = try std.fmt.bufPrint(&buff, "{s}/{s}.html", .{ "test-out", @typeName(Tree) });
        try canvas.writeHtml(test_alloc, testing.io, fpath, true);
    }
}
