const std = @import("std");
const calc = @import("calc.zig");
const vol = @import("volume.zig");
const math = std.math;
const Vec2f = calc.Vec2f;
const Box2f = vol.Box2f;

pub const Curve = enum {
    Morton, // the 'standard' Lebesgue / Morton curve produced by bit-interleaving
    Spring, // identical to Z for base = 2, but stretches for higher bases
    U, // looks like this: |_|
    Zigzag, // fancy

    const morton2_index_map = [2][2]u4{
        .{ 0, 1 },
        .{ 2, 3 },
    };

    const u2_index_map = [2][2]u4{
        .{ 0, 3 },
        .{ 1, 2 },
    };

    const u4_index_map = [4][4]u4{
        .{ 0x0, 0x3, 0xC, 0xF },
        .{ 0x1, 0x2, 0xD, 0xE },
        .{ 0x4, 0x7, 0x8, 0xA },
        .{ 0x5, 0x6, 0x9, 0xB },
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
        // TODO: can this be cleaned up with a tagged union?
        return switch (curve) {
            .Morton => if (n == 2) morton2_index_map else morton4_index_map,
            .Spring => if (n == 2) morton2_index_map else spring4_index_map,
            .U => if (n == 2) u2_index_map else u4_index_map,
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
/// Nodes on subsequent levels each have num_children.
pub fn Indexer2f(
    comptime subdivisions_per_axis: u8, // cells per axis, per level; choose 2 or 4
    comptime compressed_top_levels: u4, // levels folded into level 0; 1 = uncompressed
    comptime regular_levels: u4, // subdividing levels below level 0
    comptime curve_type: Curve, // type of space-filling curve used
) type {
    return struct {
        cell_size: f32,
        inv_cell_size: f32,
        min_pt: Vec2f,
        max_pt: Vec2f,

        comptime { // check arguments are supported when compiling
            if (!(subdivisions_per_axis == 2 or subdivisions_per_axis == 4)) {
                @compileError("subdivisions_per_axis must be 2 or 4");
            }
            if (compressed_top_levels == 0) {
                @compileError("compressed_top_levels must be greater than 0");
            }
        }

        pub const CurveIndex = math.IntFittingRange(0, num_leaves - 1);
        pub const LevelIndex = math.IntFittingRange(0, depth - 1); // 0 = top level (coarsest grid)
        pub const base = subdivisions_per_axis;
        pub const curve = curve_type;
        pub const top_levels = compressed_top_levels;
        pub const depth = 1 + regular_levels;
        pub const effective_depth = compressed_top_levels + regular_levels;
        // level 0 spans top_levels subdivisions; every level below it spans one
        pub const nodes_in_level = calc.getPow2nSequence(base, top_levels, effective_depth);
        pub const num_children = base * base;
        pub const num_leaves = nodes_in_level[depth - 1];
        const lvl_bitshift = math.log2(num_children);
        const axis_bitshift = math.log2(base);
        const coord_max: Vec2f = @splat(calc.asf32(math.sqrt(num_leaves) - 1));
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

        /// Gets the position index of a leaf node's predecessor at the identified level.
        pub fn getLeafPredecessor(leaf_index: CurveIndex, level: LevelIndex) CurveIndex {
            return leaf_index >> leafShiftToLevel(level);
        }

        /// Gets the position index of the first child node, one level below the parent.
        pub fn getFirstChild(parent_pos: CurveIndex) CurveIndex {
            return parent_pos <<| lvl_bitshift;
        }

        /// Gets the bitshift required to move a leaf index up to the identified level.
        /// From the zig language reference operator table for `a << b`:
        /// "b must be comptime-known or have a type with log2 number of bits of a"
        fn leafShiftToLevel(level: LevelIndex) math.Log2Int(CurveIndex) {
            const lvl_diff = @as(LevelIndex, @truncate(depth - 1)) - level;
            return @truncate(lvl_diff * lvl_bitshift);
        }

        /// A bounding square (not a rectangle) is fit around the provided corner points.
        pub fn init(corner_1: Vec2f, corner_2: Vec2f) !Self {
            const min_pt: Vec2f = @min(corner_1, corner_2);
            const max_pt: Vec2f = @max(corner_1, corner_2);
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
            const min = self.gitMinForGridCoords(grid_coords);
            return .{ .min = min, .max = min + @as(Vec2f, @splat(self.cell_size)) };
        }

        /// Gets a box covering the region of space for a node at any level.
        pub fn getCellBoundaryAtLevel(self: *const Self, level: LevelIndex, index: CurveIndex) Box2f {
            const first_leaf = index << leafShiftToLevel(level);
            const grid_coords = getGridCoordsForIndex(first_leaf);
            const side_len = self.cell_size * level_scales[level];
            const min = self.gitMinForGridCoords(grid_coords);
            return .{ .min = min, .max = min + @as(Vec2f, @splat(side_len)) };
        }

        /// Gets the min corner of the identified cell.
        fn gitMinForGridCoords(self: *const Self, gc: GridCoords) Vec2f {
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
            const offset_clamped = math.clamp(offset_scaled, calc.zero2f, coord_max);
            return .{ .row = @trunc(offset_clamped[1]), .col = @trunc(offset_clamped[0]) };
        }

        /// Gets the row + column number for the provided point in the top-level grid.
        fn getTopLevelCoordsForPoint(self: *const Self, point: Vec2f) GridCoords {
            const gc = self.getGridCoordsForPoint(point);
            const shift = axis_bitshift * regular_levels;
            return .{ .row = gc.row >> shift, .col = gc.col >> shift };
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

test "quad tree indexing" {
    const QuadIndexer = Indexer2f(2, 1, 2, Curve.Morton);
    var qt = try QuadIndexer.init(Vec2f{ 0, 0 }, Vec2f{ 8, 8 });
    const pt_a = Vec2f{ 4.1, 4.1 };
    const index_a = qt.getLeafIndexForPoint(pt_a);
    const parent_a = QuadIndexer.getLeafPredecessor(index_a, 1);
    const grandparent_a = QuadIndexer.getLeafPredecessor(index_a, 0);
    try testing.expectEqual(48, index_a);
    try testing.expectEqual(12, parent_a);
    try testing.expectEqual(3, grandparent_a);
    const pt_b = Vec2f{ 7.99, 7.99 };
    const ln_num_b = qt.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(QuadIndexer.num_leaves - 1, ln_num_b);

    for (0..QuadIndexer.num_leaves) |i| {
        const index: QuadIndexer.CurveIndex = @intCast(i);
        const pred_2 = QuadIndexer.getLeafPredecessor(index, 2);
        const pred_1 = QuadIndexer.getLeafPredecessor(index, 1);
        const pred_0 = QuadIndexer.getLeafPredecessor(index, 0);
        // ancestors can be identified by the MSBs of the child index
        try testing.expectEqual(index, pred_2); // same level pred = node
        try testing.expectEqual(index / 4, pred_1);
        try testing.expectEqual(index / 16, pred_0);
    }
}

test "hexa tree indexing" {
    const HexaIndexer = Indexer2f(4, 1, 1, Curve.Spring);
    var tree = try HexaIndexer.init(Vec2f{ 0, 0 }, Vec2f{ 8, 8 });
    const pt_a = Vec2f{ 4.1, 2.1 };
    const index_a = tree.getLeafIndexForPoint(pt_a);
    const parent_a = HexaIndexer.getLeafPredecessor(index_a, 0);
    try testing.expectEqual(96, index_a);
    try testing.expectEqual(6, parent_a);
    const pt_b = Vec2f{ 7.99, 7.99 };
    const index_b = tree.getLeafIndexForPoint(pt_b);
    try testing.expectEqual(HexaIndexer.num_leaves - 1, index_b);

    for (0..HexaIndexer.num_leaves) |i| {
        const index: HexaIndexer.CurveIndex = @intCast(i);
        const parent_index = HexaIndexer.getLeafPredecessor(index, 0);
        const parent_child0 = HexaIndexer.getFirstChild(parent_index);
        // ancestors can be identified by the MSBs of the child index
        try testing.expectEqual(index / 16, parent_index);
        try testing.expect(@abs(index - parent_child0) < 16);
    }
}

test "inter-leaf distances are bounded" {
    const Indexers = .{
        Indexer2f(4, 1, 2, Curve.Morton),
        Indexer2f(4, 1, 1, Curve.Spring),
        Indexer2f(4, 1, 1, Curve.Zigzag),
    };
    var prng = std.Random.DefaultPrng.init(0);
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
        Indexer2f(2, 1, 4, Curve.U),
        Indexer2f(4, 1, 1, Curve.Spring),
        Indexer2f(4, 1, 2, Curve.Zigzag),
    };
    var prng = std.Random.DefaultPrng.init(0);
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

test "draw indexer curves" {
    const Indexers = .{
        Indexer2f(2, 1, 5, Curve.Morton),
        Indexer2f(2, 1, 5, Curve.U),
        Indexer2f(4, 1, 2, Curve.Spring),
        Indexer2f(4, 1, 2, Curve.Zigzag),
    };
    const bg_style: svg.ShapeStyle = .{ .fill_active = true, .fill_hsl = .{ 0, 0, 95 } };
    const line_style: svg.ShapeStyle = .{ .stroke_width = 2, .stroke_hsl = .{ 90, 60, 40 } };
    const min_pt = Vec2f{ 0, 0 };
    const max_pt = Vec2f{ 1024, 1024 };

    // Draw some pretty pictures of the indexers's curves so they can be eyeballed.
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
    }
}
