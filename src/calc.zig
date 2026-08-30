//! This module contains core integer, float, and random functions used throughout this library.
const std = @import("std");
const math = std.math;
pub const Vec2f = @Vector(2, f32);
pub const zero2f = Vec2f{ 0, 0 };

/// Gets an array containing the range: 0, 1, ... len - 1.
pub fn getRange(comptime T: type, comptime len: u24) [len]T {
    @setEvalBranchQuota(len + 1);
    var range: [len]T = undefined;
    for (0..len) |i| range[i] = @intCast(i);
    return range;
}

/// Gets an array containing the reversed range: len - 1, len - 2, ... 0.
pub fn getReversedRange(comptime T: type, comptime len: u24) [len]T {
    @setEvalBranchQuota(len + 1);
    var range: [len]T = undefined;
    for (0..len) |i| range[i] = @intCast(len - i - 1);
    return range;
}

/// Generates the sequence S := { base ^ (2 * n) | n in {n_first, ... n_last} }.
pub fn getPow2nSequence(
    comptime base: u8,
    comptime n_first: u8,
    comptime n_last: u8,
) [1 + n_last - n_first]usize {
    var seq: [1 + n_last - n_first]usize = undefined;
    inline for (0..seq.len, n_first..) |i, n| {
        seq[i] = math.powi(usize, base, 2 * n) catch unreachable;
    }
    return seq;
}

/// Computes the 32-bit Morton code (Z-order) for a given x + y by interleaving bits.
/// Adapted from 'Bit Twiddling Hacks' by Sean Eron Anderson:
/// https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
pub fn getInterleaved(xy: [2]u16) u32 {
    // TODO: come up with a generalised version for shorter integers
    // Also, try leverage the following hardware instructions (when available):
    // https://geoff.space/2024/06/pext-and-pdep/
    var x_u32: u32 = xy[0];
    x_u32 = (x_u32 | (x_u32 << 8)) & 0x00FF00FF;
    x_u32 = (x_u32 | (x_u32 << 4)) & 0x0F0F0F0F;
    x_u32 = (x_u32 | (x_u32 << 2)) & 0x33333333;
    x_u32 = (x_u32 | (x_u32 << 1)) & 0x55555555;
    var y_u32: u32 = xy[1];
    y_u32 = (y_u32 | (y_u32 << 8)) & 0x00FF00FF;
    y_u32 = (y_u32 | (y_u32 << 4)) & 0x0F0F0F0F;
    y_u32 = (y_u32 | (y_u32 << 2)) & 0x33333333;
    y_u32 = (y_u32 | (y_u32 << 1)) & 0x55555555;
    return x_u32 | (y_u32 << 1);
}

pub fn getDeinterleaved(z: u32) [2]u16 {
    // TODO: as per above function's notes
    var x = z & 0x55555555;
    x = (x | (x >> 1)) & 0x33333333;
    x = (x | (x >> 2)) & 0x0F0F0F0F;
    x = (x | (x >> 4)) & 0x00FF00FF;
    x = (x | (x >> 8)) & 0x0000FFFF;
    // Shift z right by 1 first so y's odd bits move into the even positions
    var y = (z >> 1) & 0x55555555;
    y = (y | (y >> 1)) & 0x33333333;
    y = (y | (y >> 2)) & 0x0F0F0F0F;
    y = (y | (y >> 4)) & 0x00FF00FF;
    y = (y | (y >> 8)) & 0x0000FFFF;
    return .{ @truncate(x), @truncate(y) };
}

pub fn sortPairsLessThan(comptime T: type, pairs: [][2]T) void {
    for (pairs) |*p| if (p[0] > p[1]) std.mem.swap(T, &p[0], &p[1]);
    std.sort.pdq([2]T, pairs, {}, struct {
        fn less(_: void, a: [2]T, b: [2]T) bool {
            return std.mem.lessThan(T, &a, &b);
        }
    }.less);
}

/// Converts the integer k to a 32 bit float. Saves a bit of typing.
pub fn asf32(k: anytype) f32 {
    return @floatFromInt(k);
}

/// Returns || v ||, the Euclidean norm of the vector v.
pub fn norm(v: anytype) f32 {
    return @sqrt(squaredSum(v));
}

/// Returns < v, v >, the dot product of v with itself.
pub fn squaredSum(v: anytype) f32 {
    return dotProduct(v, v);
}

/// Returns < a, b >, the dot product a and b.
pub fn dotProduct(a: anytype, b: anytype) f32 {
    return @reduce(.Add, a * b);
}

/// Returns a scaled version of the input vector.
pub fn scaledVec(alpha: f32, v: anytype) @TypeOf(v) {
    const alpha_vec: @TypeOf(v) = @splat(alpha);
    return alpha_vec * v;
}

// solves for t that minimises || u + t * v ||
fn solveTimeThatMinimisesDist(u: Vec2f, v: Vec2f) f32 {
    const denom = dotProduct(v, v);
    return if (denom == 0) 0 else -dotProduct(u, v) / denom;
}

/// Returns (t, d_min^2) between two points moving with constant velocity (no restriction on t).
/// Each point is described by the equation: r(t) = u + t * v (u and v are vectors, t is a real number).
pub fn solveMinDistSquared(
    u_a: Vec2f,
    v_a: Vec2f,
    u_b: Vec2f,
    v_b: Vec2f,
) Vec2f {
    const u = u_a - u_b;
    const v = v_a - v_b;
    const t: f32 = solveTimeThatMinimisesDist(u, v);
    const d = u + scaledVec(t, v);
    return .{ t, squaredSum(d) };
}

/// Returns (t, d_min^2) between two points moving with constant velocity; t clamped to [t_min, t_max].
/// Each point is described by the equation: r(t) = u + t * v (u and v are vectors, t is a real number).
pub fn solveMinDistSquaredClamp(
    u_a: Vec2f,
    v_a: Vec2f,
    u_b: Vec2f,
    v_b: Vec2f,
    t_min: f32,
    t_max: f32,
) Vec2f {
    const u = u_a - u_b;
    const v = v_a - v_b;
    var t = solveTimeThatMinimisesDist(u, v);
    t = @max(t_min, @min(t, t_max));
    const d = u + scaledVec(t, v);
    return .{ t, squaredSum(d) };
}

/// Returns the squared distance between the point p and an axis-aligned bounding box.
pub fn pointBoxDistSq(p: Vec2f, box_min: Vec2f, box_max: Vec2f) f32 {
    const closest = math.clamp(p, box_min, box_max);
    return squaredSum(p - closest);
}

/// Returns the squared distance between the point p and the segment [a, b].
pub fn pointSegDistSq(p: Vec2f, a: Vec2f, b: Vec2f) f32 {
    const d = b - a;
    const denom = squaredSum(d);
    const t = if (denom == 0) 0 else math.clamp(dotProduct(p - a, d) / denom, 0, 1);
    const diff = p - (a + scaledVec(t, d));
    return squaredSum(diff);
}

/// Returns true if the line segment [start, end] intersects the axis-aligned box.
/// Uses a slab test adapted by Real-Time Collision Detection by Christer Ericson.
pub fn segmentIntersectsBox(start: Vec2f, end: Vec2f, box_min: Vec2f, box_max: Vec2f) bool {
    const d = end - start;
    var t_min: f32 = 0;
    var t_max: f32 = 1;
    inline for (0..2) |i| {
        if (d[i] == 0) {
            if (start[i] < box_min[i] or start[i] > box_max[i]) return false;
        } else {
            const inv_d = 1.0 / d[i];
            var t1 = (box_min[i] - start[i]) * inv_d;
            var t2 = (box_max[i] - start[i]) * inv_d;
            if (t1 > t2) {
                const tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            t_min = @max(t_min, t1);
            t_max = @min(t_max, t2);
            if (t_min > t_max) return false;
        }
    }
    return true;
}

/// Returns a point's coordinates relative to the provided frame.
pub fn transformToFrame(p: Vec2f, frame_origin: Vec2f, frame_axis: Vec2f) Vec2f {
    const ay = Vec2f{ -frame_axis[1], frame_axis[0] };
    const rel = p - frame_origin;
    return .{ dotProduct(rel, frame_axis), dotProduct(rel, ay) };
}

/// Gets a u64 based on system clock's measured nanoseconds - helpful in tests
pub fn getClockBasedRngSeed(io: std.Io) u64 {
    const now = std.Io.Clock.real.now(io);
    return @truncate(@abs(now.nanoseconds));
}

/// Use this in errdefer block to help reproduce an error that might be related to a random seed.
pub fn printErrorMessageForRandomSeed(seed: u64) void {
    std.debug.print("Error when testing with random data; seed = {d}\n", .{seed});
}

/// Defines a probability distribution used to generate f32 test data.
pub const ProbDensityFunc = union(enum) {
    uniform: struct { min: f32, max: f32 },
    normal: struct { mean: f32, stddev: f32 },

    /// Parses a pdf string of the form "U(min,max)" or "N(mean,stddev)".
    pub fn fromPdfString(str: []const u8) !ProbDensityFunc {
        const pdf_err = error.InvalidPdfArgs;
        if (str.len < 2 or str[1] != '(' or str[str.len - 1] != ')') return pdf_err;
        const params_str = str[2 .. str.len - 1];
        const comma_idx = std.mem.indexOfScalar(u8, params_str, ',');
        if (comma_idx == null) return error.InvalidPdfArgs;
        const a = try std.fmt.parseFloat(f32, params_str[0..comma_idx.?]);
        const b = try std.fmt.parseFloat(f32, params_str[comma_idx.? + 1 ..]);
        return switch (std.ascii.toUpper(str[0])) {
            'N' => if (a <= 0) pdf_err else .{ .normal = .{ .mean = a, .stddev = b } },
            'U' => if (a >= b) pdf_err else .{ .uniform = .{ .min = a, .max = b } },
            else => return error.UnrecognisedPdfType,
        };
    }

    pub fn getFloat(pdf: ProbDensityFunc, random: std.Random) f32 {
        return switch (pdf) {
            .normal => |n| n.mean + n.stddev * random.floatNorm(f32),
            .uniform => |u| u.min + (u.max - u.min) * random.float(f32),
        };
    }

    // Fills the provided slice with random floats according to the distribution.
    pub fn fillFloat(pdf: ProbDensityFunc, random: std.Random, slice: []f32) void {
        for (0..slice.len) |i| slice[i] = getFloat(pdf, random);
    }

    // Fills the provided slice with vectors with random coefficients (symmetric in both axes).
    pub fn fillVec2f(pdf: ProbDensityFunc, random: std.Random, vec_slice: []Vec2f) void {
        return fillFloat(pdf, random, @ptrCast(vec_slice));
    }
};

const testing = std.testing;
const tolerance = 0.0001;

test "pow-2n sequence" {
    try testing.expectEqual([_]usize{ 4, 16, 64, 256 }, getPow2nSequence(2, 1, 4));
    try testing.expectEqual([_]usize{ 16, 256, 4096 }, getPow2nSequence(4, 1, 3));
    try testing.expectEqual([_]usize{ 64, 4096 }, getPow2nSequence(8, 1, 2));
    // the last entry is the leaf count, and the entries are base^2 apart
    const seq = getPow2nSequence(2, 1, 5);
    try testing.expectEqual(1024, seq[seq.len - 1]);
    for (seq[1..], seq[0 .. seq.len - 1]) |cur, prev| {
        try testing.expectEqual(4 * prev, cur);
    }
}

test "vec norm" {
    const u = Vec2f{ 1, 0 };
    const v = Vec2f{ 0, 1 };
    const w = Vec2f{ 1, 1 };

    try testing.expectApproxEqAbs(1.0, norm(u), tolerance);
    try testing.expectApproxEqAbs(1.0, norm(v), tolerance);
    try testing.expectApproxEqAbs(math.sqrt2, norm(w), tolerance);
}

test "closest dist" {
    const pos_a = Vec2f{ 1, 0 };
    const pos_b = Vec2f{ 0, 1 };

    const zero_result = solveMinDistSquared(zero2f, zero2f, zero2f, zero2f);
    try testing.expectApproxEqAbs(0, zero_result[1], tolerance);

    const static_result = solveMinDistSquared(pos_a, zero2f, pos_b, zero2f);
    try testing.expectApproxEqAbs(2, static_result[1], tolerance);

    const parallel_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 });
    try testing.expectApproxEqAbs(1, parallel_result[1], tolerance);

    const opposite_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 });
    try testing.expectApproxEqAbs(1, opposite_result[1], tolerance);

    const perpendicular_result = solveMinDistSquared(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 });
    try testing.expectApproxEqAbs(0, perpendicular_result[1], tolerance);
}

test "closest dist interval" {
    const pos_a = Vec2f{ 1, 0 };
    const pos_b = Vec2f{ 0, 1 };

    const parallel_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 2, 0 }, 0, 100);
    try testing.expectApproxEqAbs(1, parallel_result[1], tolerance);

    const opposite_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ -1, 0 }, -1.5, -0.5);
    try testing.expectApproxEqAbs(1, opposite_result[1], tolerance);

    const perpendicular_result = solveMinDistSquaredClamp(pos_a, Vec2f{ 1, 0 }, pos_b, Vec2f{ 0, 1 }, -3, 3);
    try testing.expectApproxEqAbs(0, perpendicular_result[1], tolerance);
}

test "morton interleaving is 1:1" {
    // should be a bijection for grid coords (row + col)
    const seed = getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer std.debug.print("Error when testing with random data; seed = {d}\n", .{seed});
    const random = prng.random();
    for (0..5000) |_| {
        const x = random.int(u16);
        const y = random.int(u16);
        const z = getInterleaved(.{ x, y });
        const xy = getDeinterleaved(z);
        try std.testing.expectEqual(x, xy[0]);
        try std.testing.expectEqual(y, xy[1]);
    }
}
