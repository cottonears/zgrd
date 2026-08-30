const std = @import("std");
const calc = @import("calc.zig");
const data = @import("data.zig");
const vol = @import("volume.zig");
const math = std.math;
const Vec2f = calc.Vec2f;
const Box2f = vol.Box2f;

pub const Curve = enum {
    // TODO: add support for Hilbert curves!
    Morton, // the standard Lebesgue / Morton Z-shaped curve produced by bit-interleaving
    Spring, // same Z-shape as Morton for base = 2, but stretches for higher bases
    Zigzag, // fancy

    const morton2_index_map = [2][2]u4{
        .{ 0, 1 },
        .{ 2, 3 },
    };

    const morton4_index_map = [4][4]u4{
        .{ 0x0, 0x1, 0x4, 0x5 },
        .{ 0x2, 0x3, 0x6, 0x7 },
        .{ 0x8, 0x9, 0xC, 0xD },
        .{ 0xA, 0xB, 0xE, 0xF },
    };

    const spring4_index_map = [4][4]u4{
        .{ 0x0, 0x1, 0x2, 0x3 },
        .{ 0x4, 0x5, 0x6, 0x7 },
        .{ 0x8, 0x9, 0xA, 0xB },
        .{ 0xC, 0xD, 0xE, 0xF },
    };

    const zigzag4_index_map = [4][4]u4{
        .{ 0x0, 0x1, 0x5, 0x6 },
        .{ 0x2, 0x4, 0x7, 0xC },
        .{ 0x3, 0x8, 0xB, 0xD },
        .{ 0x9, 0xA, 0xE, 0xF },
    };

    fn getCurveIndexMap(comptime n: comptime_int, comptime curve: Curve) [n][n]u4 {
        return switch (curve) {
            .Morton => if (n == 2) morton2_index_map else morton4_index_map,
            .Spring => if (n == 2) morton2_index_map else spring4_index_map,
            .Zigzag => if (n == 4) zigzag4_index_map else {
                @compileError("zigzag only supports base 4");
            },
        };
    }

    fn getInverseIndexMap(comptime n: comptime_int, comptime curve: Curve) [n * n][2]u4 {
        const fwd_map = getCurveIndexMap(n, curve);
        var inv_map: [n * n][2]u4 = undefined;
        for (0..n) |i| {
            for (0..n) |j| {
                const index = fwd_map[i][j];
                inv_map[index] = .{ @intCast(i), @intCast(j) };
            }
        }
        return inv_map;
    }
};

/// Recursively indexes a region on the 2D plane.
/// Level 0 'compresses' several layers' worth of children to limit traversal depth.
/// Nodes on subsequent levels each have sub_divs x sub_divs children.
pub fn Indexer2f(
    comptime subdivs_per_axis: u8, // cells per axis, per level; choose 2 or 4
    comptime top_lvl_compression: u4, // levels folded into level 0; 1 = uncompressed
    comptime regular_levels: u4, // uncompressed levels below level 0
    comptime curve_type: Curve, // type of space-filling curve used
) type {
    return struct {
        cell_size: f32,
        inv_cell_size: f32,
        min_pt: Vec2f,
        max_pt: Vec2f,

        comptime { // check arguments are supported when compiling
            if (!(subdivs_per_axis == 2 or subdivs_per_axis == 4)) {
                @compileError("subdivisions_per_axis must be 2 or 4");
            }
            if (top_lvl_compression == 0) {
                @compileError("compressed_top_levels must be greater than 0");
            }
        }

        pub const CurveIndex = math.IntFittingRange(0, num_leaves - 1);
        pub const LevelIndex = math.IntFittingRange(0, depth - 1); // 0 = top level
        pub const base = subdivs_per_axis;
        pub const curve = curve_type;
        pub const top_levels = top_lvl_compression;
        pub const depth = 1 + regular_levels;
        pub const effective_depth = top_lvl_compression + regular_levels;
        pub const nodes_in_level = calc.getPow2nSequence(base, top_levels, effective_depth);
        pub const num_children = base * base;
        pub const num_leaves = nodes_in_level[depth - 1];
        pub const type_label = std.fmt.comptimePrint(
            "{s} {d} x {d} [{d}]",
            .{ @tagName(curve), base, effective_depth, depth },
        );
        const lvl_bitshift = math.log2(num_children);
        const axis_bitshift = math.log2(base);
        const coord_max: u16 = @intCast(math.sqrt(num_leaves) - 1);
        const coord_max_vec2f: Vec2f = @splat(calc.asf32(coord_max));
        const level_scales = blk: {
            var scales: [depth]f32 = undefined;
            for (&scales, 0..) |*s, lvl| {
                s.* = calc.asf32(math.powi(usize, base, depth - lvl - 1) catch unreachable);
            }
            break :blk scales;
        };
        const index_map = Curve.getCurveIndexMap(base, curve_type);
        const grid_coord_map = Curve.getInverseIndexMap(base, curve_type);
        const GridIndex = u16; // NOTE: any smaller is slower (stored as register temporary).
        const GridCoords = struct { row: GridIndex, col: GridIndex };
        const Self = @This();

        /// Gets the position index of the first child node, one level below the parent.
        pub fn getFirstChild(parent_pos: CurveIndex) CurveIndex {
            return parent_pos <<| lvl_bitshift;
        }

        /// Gets the position index of the first leaf successor of the parent.
        pub fn getFirstLeafSuccessor(lvl: LevelIndex, node: CurveIndex) CurveIndex {
            return node <<| leafShiftToLevel(lvl);
        }

        /// Gets the number of leaf successors below a node at the specified level.
        pub fn getNumberLeafSuccessors(lvl: LevelIndex) usize {
            return @as(CurveIndex, 1) <<| leafShiftToLevel(lvl);
        }

        /// Gets the position index of a leaf node's predecessor at the identified level.
        pub fn getLeafPredecessor(leaf_index: CurveIndex, level: LevelIndex) CurveIndex {
            return leaf_index >> leafShiftToLevel(level);
        }

        /// Gets the row + column number for the provided index.
        /// Map from curve index -> grid coords.
        fn getGridCoordsForIndex(leaf_index: CurveIndex) GridCoords {
            var row: GridIndex = 0;
            var col: GridIndex = 0;
            inline for (0..effective_depth) |i| {
                const lvl_diff = effective_depth - i - 1;
                const index_i = (leaf_index >> lvl_diff * lvl_bitshift) & (num_children - 1);
                const grid_coords = grid_coord_map[index_i];
                row = (row << axis_bitshift) + @as(GridIndex, @truncate(grid_coords[0]));
                col = (col << axis_bitshift) + @as(GridIndex, @truncate(grid_coords[1]));
            }
            return .{ .row = row, .col = col };
        }

        /// Gets the curve index for the provided point in the leaf-level grid.
        /// Map from grid coords -> curve index.
        fn getIndexForGridCoords(comptime digits: u8, c: GridCoords) CurveIndex {
            var index: CurveIndex = 0;
            inline for (0..digits) |i| {
                const lvl_diff = digits - i - 1;
                const lvl_row = (c.row >> axis_bitshift * lvl_diff) & (base - 1);
                const lvl_col = (c.col >> axis_bitshift * lvl_diff) & (base - 1);
                const index_i: CurveIndex = @intCast(index_map[lvl_row][lvl_col]);
                index = (index << lvl_bitshift) + index_i;
            }
            return index;
        }

        /// Gets the bitshift required to move a leaf index up to the identified level.
        fn leafShiftToLevel(level: LevelIndex) math.Log2Int(CurveIndex) {
            const lvl_diff = @as(LevelIndex, @truncate(depth - 1)) - level;
            return @truncate(lvl_diff * lvl_bitshift);
        }

        /// A bounding square (not a rectangle) is fit around the provided corner points.
        pub fn init(corner_1: Vec2f, corner_2: Vec2f) !Self {
            const min_pt = @min(corner_1, corner_2);
            const max_pt = @max(corner_1, corner_2);
            const size = @max(max_pt[0] - min_pt[0], max_pt[1] - min_pt[1]);
            if (size <= 0.0) return error.InvalidSize;
            var lvl_size = size;
            for (0..effective_depth) |_| lvl_size /= base;
            return Self{
                .cell_size = lvl_size,
                .inv_cell_size = 1.0 / lvl_size,
                .min_pt = min_pt,
                .max_pt = max_pt,
            };
        }

        /// Gets the index of the leaf node that the query point lies within.
        /// Map from R^2 -> grid coords -> index space
        pub fn getLeafIndexForPoint(self: *const Self, point: Vec2f) CurveIndex {
            const grid_coord = self.getGridCoordsForPoint(point);
            return getIndexForGridCoords(effective_depth, grid_coord);
        }

        /// Gets the indexes of top-level cells that lie within the box b.
        pub fn getTopLevelIndexesForBox(self: *const Self, buff: []CurveIndex, b: Box2f) []CurveIndex {
            const lo = self.getTopLevelCoordsForPoint(b.min);
            const hi = self.getTopLevelCoordsForPoint(b.max);
            var len: usize = 0;
            for (lo.row..hi.row + 1) |row| {
                for (lo.col..hi.col + 1) |col| {
                    const coords: GridCoords = .{ .row = @intCast(row), .col = @intCast(col) };
                    buff[len] = getIndexForGridCoords(top_levels, coords);
                    len += 1;
                }
            }
            return buff[0..len];
        }

        /// Gets a box covering the region of space for the indexed leaf node.
        /// Map from index space -> grid coords -> cell extents (subsets of R^2).
        pub fn getLeafCellBoundary(self: *const Self, leaf_index: CurveIndex) Box2f {
            const grid_coords = getGridCoordsForIndex(leaf_index);
            const min = self.getMinForGridCoords(grid_coords);
            return .{ .min = min, .max = min + @as(Vec2f, @splat(self.cell_size)) };
        }

        /// Gets a box covering the region of space for a node at any level.
        pub fn getCellBoundaryAtLevel(self: *const Self, level: LevelIndex, index: CurveIndex) Box2f {
            const first_leaf = index << leafShiftToLevel(level);
            const grid_coords = getGridCoordsForIndex(first_leaf);
            const side_len = self.cell_size * level_scales[level];
            const min = self.getMinForGridCoords(grid_coords);
            return .{ .min = min, .max = min + @as(Vec2f, @splat(side_len)) };
        }

        /// Gets the indexes of leaf cells that are n distance (taxi-cab metric) from a leaf node.
        pub fn getLeafCellNeighbours(buff: []CurveIndex, index: CurveIndex, n: u16) ![]CurveIndex {
            var blen: usize = 0;
            if (n == 0) {
                buff[0] = index;
                return buff[0..1];
            }
            const grid_coords = getGridCoordsForIndex(index);
            const top = grid_coords.row -| n;
            const bot: GridIndex = @min(coord_max, grid_coords.row +| n);
            const left = grid_coords.col -| n;
            const right: GridIndex = @min(coord_max, grid_coords.col +| n);
            // vertical scans
            const add_left = grid_coords.col - left == n;
            const add_right = right - grid_coords.col == n;
            for (top..bot + 1) |i| {
                if (add_left) {
                    const l = GridCoords{ .row = @intCast(i), .col = left };
                    buff[blen] = getIndexForGridCoords(effective_depth, l);
                    blen += 1;
                }
                if (add_right) {
                    const r = GridCoords{ .row = @intCast(i), .col = right };
                    buff[blen] = getIndexForGridCoords(effective_depth, r);
                    blen += 1;
                }
            }
            // horizontal scans
            const add_top = grid_coords.row - top == n;
            const add_bot = bot - grid_coords.row == n;
            const h_start = if (add_left) left + 1 else left;
            const h_end = if (add_right) right else right + 1;
            for (h_start..h_end) |j| {
                if (add_top) {
                    const t = GridCoords{ .row = top, .col = @intCast(j) };
                    buff[blen] = getIndexForGridCoords(effective_depth, t);
                    blen += 1;
                }
                if (add_bot) {
                    const b = GridCoords{ .row = bot, .col = @intCast(j) };
                    buff[blen] = getIndexForGridCoords(effective_depth, b);
                    blen += 1;
                }
            }
            const idx_buff = buff[0..blen];
            std.sort.pdq(CurveIndex, idx_buff, {}, std.sort.asc(CurveIndex));
            return idx_buff;
        }

        /// Gets the min corner of the identified cell.
        fn getMinForGridCoords(self: *const Self, gc: GridCoords) Vec2f {
            return .{
                self.min_pt[0] + gc.col * self.cell_size,
                self.min_pt[1] + gc.row * self.cell_size,
            };
        }

        /// Gets the row + column number for the provided point in the leaf-level grid.
        /// Map from R^2 -> grid coords.
        fn getGridCoordsForPoint(self: *const Self, point: Vec2f) GridCoords {
            const offset = point - self.min_pt;
            const offset_scaled = calc.scaledVec(self.inv_cell_size, offset);
            const offset_clamped = math.clamp(offset_scaled, calc.zero2f, coord_max_vec2f);
            return .{ .row = @trunc(offset_clamped[1]), .col = @trunc(offset_clamped[0]) };
        }

        /// Gets the row + column number for the provided point in the top-level grid.
        fn getTopLevelCoordsForPoint(self: *const Self, point: Vec2f) GridCoords {
            const gc = self.getGridCoordsForPoint(point);
            const shift = axis_bitshift * regular_levels;
            return .{ .row = gc.row >> shift, .col = gc.col >> shift };
        }
    };
}

const svg = @import("svg.zig");
const testing = std.testing;
const test_alloc = std.testing.allocator;
const test_dist: calc.ProbDensityFunc = .{ .uniform = .{ .min = -5.0, .max = 5.0 } };
var test_dir = "test-out";

test "check coord map lookups are 1:1" {
    const spring4_index_map = Curve.getCurveIndexMap(4, .Spring);
    const zigzag4_index_map = Curve.getCurveIndexMap(4, .Zigzag);
    const spring_grid_lookup = Curve.getInverseIndexMap(4, .Spring);
    const zigzag_grid_lookup = Curve.getInverseIndexMap(4, .Zigzag);
    for (0..4) |i| {
        for (0..4) |j| {
            const s = spring4_index_map[i][j];
            const z = zigzag4_index_map[i][j];
            const s_inv = spring_grid_lookup[s];
            const z_inv = zigzag_grid_lookup[z];
            try std.testing.expectEqual(s_inv[0], i);
            try std.testing.expectEqual(s_inv[1], j);
            try std.testing.expectEqual(z_inv[0], i);
            try std.testing.expectEqual(z_inv[1], j);
        }
    }
}

test "hexa tree indexing" {
    const Indexer = Indexer2f(4, 1, 1, Curve.Spring);
    var hex_indexer = try Indexer.init(.{ 0, 0 }, .{ 8, 8 });
    const pt_a = Vec2f{ 4.1, 2.1 };
    const index_a = hex_indexer.getLeafIndexForPoint(pt_a);
    const parent_a = Indexer.getLeafPredecessor(index_a, 0);
    try testing.expectEqual(96, index_a);
    try testing.expectEqual(6, parent_a);
    const pt_b = Vec2f{ 7.99, 7.99 };
    const index_b = hex_indexer.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(Indexer.num_leaves - 1, index_b);

    for (0..Indexer.num_leaves) |i| {
        const index: Indexer.CurveIndex = @intCast(i);
        const parent_index = Indexer.getLeafPredecessor(index, 0);
        const parent_child0 = Indexer.getFirstChild(parent_index);
        // ancestors can be identified by the MSBs of the child index
        try testing.expectEqual(index / 16, parent_index);
        try testing.expect(@abs(index - parent_child0) < 16);
    }
}

test "quad tree indexing" {
    const Indexer = Indexer2f(2, 1, 2, Curve.Morton);
    var quad_indexer = try Indexer.init(.{ 0, 0 }, .{ 8, 8 });
    const pt_a = Vec2f{ 4.1, 4.1 };
    const index_a = quad_indexer.getLeafIndexForPoint(pt_a);
    const parent_a = Indexer.getLeafPredecessor(index_a, 1);
    const grandparent_a = Indexer.getLeafPredecessor(index_a, 0);
    try testing.expectEqual(48, index_a);
    try testing.expectEqual(12, parent_a);
    try testing.expectEqual(3, grandparent_a);
    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = quad_indexer.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(Indexer.num_leaves - 1, ln_num_b);

    for (0..Indexer.num_leaves) |i| {
        const index: Indexer.CurveIndex = @intCast(i);
        const pred_2 = Indexer.getLeafPredecessor(index, 2);
        const pred_1 = Indexer.getLeafPredecessor(index, 1);
        const pred_0 = Indexer.getLeafPredecessor(index, 0);
        // ancestors can be identified by the MSBs of the child index
        try testing.expectEqual(index, pred_2); // same level pred = node
        try testing.expectEqual(index / 4, pred_1);
        try testing.expectEqual(index / 16, pred_0);
    }
}

test "compressed indexing test" {
    const RegIndexer = Indexer2f(4, 1, 2, Curve.Zigzag);
    const CompIndexer = Indexer2f(4, 2, 1, Curve.Zigzag);
    const CurveIndex = RegIndexer.CurveIndex;
    // same effective depth and curve -> same total leaves in each tree
    try testing.expectEqual(RegIndexer.num_leaves, CompIndexer.num_leaves);
    // compression should not affect number of children per node (only the node count on level 0)
    try testing.expectEqual(RegIndexer.num_children, CompIndexer.num_children);
    // number of successors under level 0 nodes should be reduced by compression
    const num_reg_successors = RegIndexer.getNumberLeafSuccessors(0);
    const num_comp_successors = CompIndexer.getNumberLeafSuccessors(0);
    try testing.expectEqual(num_reg_successors / RegIndexer.num_children, num_comp_successors);
    // check every index appears as a successor of a level-0 node in each tree, and order matches
    var reg_succs = try std.ArrayList(CurveIndex).initCapacity(test_alloc, RegIndexer.num_leaves);
    defer reg_succs.deinit(test_alloc);
    for (0..RegIndexer.nodes_in_level[0]) |i| {
        const reg_start = RegIndexer.getFirstLeafSuccessor(0, @truncate(i));
        const reg_end = reg_start + RegIndexer.getNumberLeafSuccessors(0);
        for (reg_start..reg_end) |j| reg_succs.appendAssumeCapacity(@truncate(j));
    }
    var comp_succs = try std.ArrayList(CurveIndex).initCapacity(test_alloc, CompIndexer.num_leaves);
    defer comp_succs.deinit(test_alloc);
    for (0..CompIndexer.nodes_in_level[0]) |i| {
        const comp_start = CompIndexer.getFirstLeafSuccessor(0, @truncate(i));
        const comp_end = comp_start + CompIndexer.getNumberLeafSuccessors(0);
        for (comp_start..comp_end) |j| comp_succs.appendAssumeCapacity(@truncate(j));
    }
    try testing.expectEqualSlices(CurveIndex, reg_succs.items, comp_succs.items);
    // finally, generate some random points and check their indexes are identical in both trees
    var reg_indexer = try RegIndexer.init(.{ 0, 0 }, .{ 8, 8 });
    var comp_indexer = try CompIndexer.init(.{ 0, 0 }, .{ 8, 8 });
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var test_pts: [1000]Vec2f = undefined;
    calc.ProbDensityFunc.fillVec2f(test_dist, prng.random(), &test_pts);
    for (test_pts) |p| {
        const r_idx = reg_indexer.getLeafIndexForPoint(p);
        const c_idx = comp_indexer.getLeafIndexForPoint(p);
        try testing.expectEqual(r_idx, c_idx);
    }
}

test "inter-leaf distances are bounded" {
    const Indexers = .{
        Indexer2f(4, 1, 2, Curve.Morton),
        Indexer2f(4, 1, 1, Curve.Spring),
        Indexer2f(4, 1, 1, Curve.Zigzag),
    };
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var test_pts: [500]Vec2f = undefined;
    calc.ProbDensityFunc.fillVec2f(test_dist, prng.random(), &test_pts);
    const min_pt = Vec2f{ -5, -5 };
    const max_pt = Vec2f{ 5, 5 };
    // check that any points (within the region) who share an index are relatively close
    inline for (Indexers) |Indexer| {
        const PointIndexPair = struct {
            index: Indexer.CurveIndex,
            point: Vec2f,
            fn lessThan(_: void, a: @This(), b: @This()) bool {
                return a.index < b.index;
            }
        };
        const idx = try Indexer.init(min_pt, max_pt);
        const max_expected_cell_dist = @sqrt(2.0) * idx.cell_size;
        var pt_idx_pairs: [test_pts.len]PointIndexPair = undefined;
        for (test_pts, 0..) |p, i| {
            pt_idx_pairs[i] = .{ .index = idx.getLeafIndexForPoint(p), .point = p };
        }
        std.sort.pdq(PointIndexPair, &pt_idx_pairs, {}, PointIndexPair.lessThan);
        // measure distance between all points that share a leaf index
        var max_measured_cell_dist: f32 = 0;
        var current_leaf_start: usize = 0;
        for (pt_idx_pairs, 0..) |pair, i| {
            if (i > 0 and pair.index != pt_idx_pairs[i - 1].index) {
                current_leaf_start = i;
            }
            for (current_leaf_start..i) |j| {
                const dist = calc.norm(pt_idx_pairs[j].point - pair.point);
                max_measured_cell_dist = @max(max_measured_cell_dist, dist);
            }
        }
        try testing.expect(max_measured_cell_dist <= max_expected_cell_dist);
    }
}

test "leaf index round trip" {
    const Indexers = .{
        Indexer2f(2, 1, 2, Curve.Morton),
        Indexer2f(4, 1, 1, Curve.Spring),
        Indexer2f(4, 1, 2, Curve.Zigzag),
    };
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var random_pts: [2]Vec2f = undefined;
    calc.ProbDensityFunc.fillVec2f(test_dist, prng.random(), &random_pts);
    const min_pt = Vec2f{ -5, -5 };
    const max_pt = Vec2f{ 5, 5 };
    // check that every leaf tiles part of the region, and its centre indexes back to it
    inline for (Indexers) |Indexer| {
        const indexer = try Indexer.init(min_pt, max_pt);
        const region = Box2f{ .min = indexer.min_pt, .max = indexer.max_pt };
        for (0..Indexer.num_leaves) |i| {
            const leaf: Indexer.CurveIndex = @intCast(i);
            const cell = indexer.getLeafCellBoundary(leaf);
            const centre = cell.getCentre();
            const centre_index = indexer.getLeafIndexForPoint(centre);
            try testing.expect(vol.checkVolumesOverlap(region, cell));
            try testing.expect(@reduce(.And, cell.min >= region.min));
            try testing.expect(@reduce(.And, cell.max <= region.max));
            try testing.expect(@reduce(.And, centre > region.min));
            try testing.expect(@reduce(.And, centre < region.max));
            try testing.expectEqual(leaf, centre_index);
        }
    }
}

test "check get leaf cell neighbours in centre" {
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var random_pts: [100]Vec2f = undefined;
    calc.ProbDensityFunc.fillVec2f(test_dist, prng.random(), &random_pts);
    const Indexer = Indexer2f(4, 1, 1, Curve.Zigzag);
    const indexer = try Indexer.init(.{ -10, -10 }, .{ 10, 10 });
    // check size and values match what is expected
    for (random_pts) |p| {
        const p_idx = indexer.getLeafIndexForPoint(p);
        const p_gc = indexer.getGridCoordsForPoint(p);
        var n_buff: [24]Indexer.CurveIndex = undefined;
        const near_p_0 = try Indexer.getLeafCellNeighbours(&n_buff, p_idx, 0);
        try testing.expectEqual(1, near_p_0.len);
        try testing.expectEqual(p_idx, near_p_0[0]);
        for (1..4) |n| {
            const near_p_n = try Indexer.getLeafCellNeighbours(&n_buff, p_idx, @intCast(n));
            try testing.expectEqual(8 * n, near_p_n.len);
            for (near_p_n) |i| {
                const i_gc = Indexer.getGridCoordsForIndex(i);
                const row_diff = if (i_gc.row > p_gc.row) i_gc.row - p_gc.row else p_gc.row - i_gc.row;
                const col_diff = if (i_gc.col > p_gc.col) i_gc.col - p_gc.col else p_gc.col - i_gc.col;
                try testing.expectEqual(n, @max(row_diff, col_diff));
            }
        }
    }
}

test "check get leaf cell neighbours near edge" {
    const Indexer = Indexer2f(4, 1, 2, Curve.Zigzag);
    const p_idx: Indexer.CurveIndex = 0;
    const p_gc = Indexer.getGridCoordsForIndex(p_idx);
    var idx_seen = [_]bool{false} ** Indexer.num_leaves;
    var n_buff: [Indexer.num_leaves]Indexer.CurveIndex = undefined;
    var n: u16 = 0;
    // search for successive rings of nearby indexes; iterate to cover the whole grid
    while (n <= Indexer.coord_max) : (n += 1) {
        const ring = try Indexer.getLeafCellNeighbours(&n_buff, p_idx, n);
        for (ring) |i| {
            const i_gc = Indexer.getGridCoordsForIndex(i);
            const row_diff = if (i_gc.row > p_gc.row) i_gc.row - p_gc.row else p_gc.row - i_gc.row;
            const col_diff = if (i_gc.col > p_gc.col) i_gc.col - p_gc.col else p_gc.col - i_gc.col;
            try testing.expectEqual(n, @max(row_diff, col_diff));
            try testing.expectEqual(false, idx_seen[i]); // indexes should be found at most once
            idx_seen[i] = true;
        }
    }
    // every leaf index should have been found at least once
    for (idx_seen) |s| try testing.expectEqual(true, s);
}

test "draw indexer curves" {
    const Indexers = .{
        Indexer2f(2, 2, 4, Curve.Morton),
        Indexer2f(4, 1, 2, Curve.Spring),
        Indexer2f(4, 1, 2, Curve.Zigzag),
    };
    const bg_style: svg.ShapeStyle = .{ .fill_active = true, .fill_hsl = .{ 0, 0, 95 } };
    const line_style: svg.ShapeStyle = .{ .stroke_width = 2, .stroke_hsl = .{ 90, 60, 40 } };
    const min_pt = Vec2f{ 0, 0 };
    const max_pt = Vec2f{ 1024, 1024 };
    // draw some pretty pictures of the indexers' curves so they can be eyeballed.
    inline for (Indexers) |Indexer| {
        const idx = try Indexer.init(min_pt, max_pt);
        var pts: [Indexer.num_leaves]Vec2f = undefined;
        var curve_lenth: f32 = 0.0;
        for (0..pts.len) |i| {
            const cell = idx.getLeafCellBoundary(@truncate(i));
            pts[i] = cell.getCentre();
            if (i > 0) {
                curve_lenth += calc.norm(pts[i] - pts[i - 1]);
            }
        }
        var test_canvas = try svg.Canvas.init(test_alloc, min_pt, max_pt, bg_style);
        defer test_canvas.deinit(test_alloc);
        try test_canvas.addPolyline(test_alloc, &pts, line_style);
        var sbuff: [128]u8 = undefined;
        const length_str = try std.fmt.bufPrint(&sbuff, "length = {d:.1}\n", .{curve_lenth});
        const text_loc = min_pt + calc.scaledVec(0.5, max_pt - min_pt);
        try test_canvas.addText(test_alloc, text_loc, length_str, 20, .{ 0, 0, 0 });
        const fpath = try std.fmt.bufPrint(&sbuff, "{s}/{any}.html", .{ test_dir, Indexer });
        try test_canvas.writeHtml(test_alloc, testing.io, fpath, true);
        // operator should inspect the output: expect("looks good to me")
    }
}
