//! This module contains definitions of volume types and related functions.
const std = @import("std");
const calc = @import("calc.zig");
const math = std.math;
const ProbDensityFunc = calc.ProbDensityFunc;
const Vec2f = calc.Vec2f;

pub const empty_ball = Ball2f{
    .centre = @splat(math.floatMax(f32)),
    .radius = -math.floatMax(f32),
};

pub const empty_box = Box2f{
    .min = @splat(math.floatMax(f32)),
    .max = @splat(-math.floatMax(f32)),
};

/// A circular region in 2D Euclidean space.
pub const Ball2f = struct {
    centre: Vec2f,
    radius: f32,
    const Self = @This();

    pub fn getBoundingBall(self: Self) Ball2f {
        return self;
    }

    pub fn getBoundingBox(self: Self) Box2f {
        if (self.isEmpty()) return empty_box;
        const disp = @as(Vec2f, @splat(self.radius));
        return .{ .min = self.centre - disp, .max = self.centre + disp };
    }

    pub fn getCentre(self: Self) Vec2f {
        return self.centre;
    }

    // TODO: change to getTransformed and allow for translation + scale
    pub fn getScaled(self: Self, factor: f32) Ball2f {
        return .{
            .centre = self.centre,
            .radius = @abs(factor) * self.radius,
        };
    }

    pub fn isEmpty(self: Self) bool {
        return self.radius <= 0;
    }
};

/// An axis-aligned rectangular region in 2D Euclidean space.
pub const Box2f = struct {
    min: Vec2f,
    max: Vec2f,
    const Self = @This();

    pub fn getBoundingBall(self: Self) Ball2f {
        if (self.isEmpty()) return empty_ball;
        const dims = self.max - self.min;
        return .{
            .centre = self.getCentre(),
            .radius = @max(dims[0], dims[1]) / 2,
        };
    }

    pub fn getBoundingBox(self: Self) Box2f {
        return self;
    }

    pub fn getCentre(self: Self) Vec2f {
        return calc.scaledVec(0.5, self.min + self.max);
    }

    // TODO: change to getTransformed and allow for translation + scale
    pub fn getScaled(self: Self, factor: f32) Box2f {
        const c = self.getCentre();
        const h = calc.scaledVec(0.5 * factor, self.max - self.min); // note the 0.5
        return .{ .min = c - h, .max = c + h };
    }

    pub fn isEmpty(self: Self) bool {
        return @reduce(.And, self.max <= self.min);
    }
};

/// An oriented bounding box.
pub const OrientedBox2f = struct {
    centre: Vec2f,
    half_extents: Vec2f,
    axis: Vec2f, // unit vector along the box's local x-axis
    const Self = @This();

    pub fn getBoundingBall(self: Self) Ball2f {
        if (self.isEmpty()) return empty_ball;
        return .{ .centre = self.centre, .radius = calc.norm(self.half_extents) };
    }

    pub fn getBoundingBox(self: Self) Box2f {
        if (self.isEmpty()) return empty_box;
        const ay = Vec2f{ -self.axis[1], self.axis[0] };
        const extent = calc.scaledVec(self.half_extents[0], @abs(self.axis)) +
            calc.scaledVec(self.half_extents[1], @abs(ay));
        return .{ .min = self.centre - extent, .max = self.centre + extent };
    }

    pub fn getCorners(self: Self) [4]Vec2f {
        const ay = Vec2f{ -self.axis[1], self.axis[0] };
        const ex = calc.scaledVec(self.half_extents[0], self.axis);
        const ey = calc.scaledVec(self.half_extents[1], ay);
        return .{
            self.centre + ex + ey,
            self.centre - ex + ey,
            self.centre - ex - ey,
            self.centre + ex - ey,
        };
    }

    pub fn getCentre(self: Self) Vec2f {
        return self.centre;
    }

    // TODO: change to getTransformed and allow for translation + scale
    pub fn getScaled(self: Self, factor: f32) OrientedBox2f {
        return .{
            .centre = self.centre,
            .half_extents = calc.scaledVec(@abs(factor), self.half_extents),
            .axis = self.axis,
        };
    }

    pub fn isEmpty(self: Self) bool {
        return self.half_extents[0] <= 0 or self.half_extents[1] <= 0;
    }
};

/// A zero-width line segment: query only, can't be stored in a tree.
pub const Line2f = struct {
    start: Vec2f,
    end: Vec2f,
    const Self = @This();

    pub fn getBoundingBox(self: Self) Box2f {
        return .{ .min = @min(self.start, self.end), .max = @max(self.start, self.end) };
    }
};

/// Returns true if the two volumes overlap.
pub fn checkVolumesOverlap(a: anytype, b: anytype) bool {
    return switch (@TypeOf(a)) {
        Ball2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapBallBall(a, b),
            Box2f => checkOverlapBallBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxBall(b, a),
            Line2f => checkOverlapLineBall(b, a),
            else => unreachable,
        },
        Box2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapBallBox(b, a),
            Box2f => checkOverlapBoxBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxBox(b, a),
            Line2f => checkOverlapLineBox(b, a),
            else => unreachable,
        },
        OrientedBox2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapOrientedBoxBall(a, b),
            Box2f => checkOverlapOrientedBoxBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxOrientedBox(a, b),
            Line2f => checkOverlapLineOrientedBox(b, a),
            else => unreachable,
        },
        Line2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapLineBall(a, b),
            Box2f => checkOverlapLineBox(a, b),
            OrientedBox2f => checkOverlapLineOrientedBox(a, b),
            else => unreachable,
        },
        else => unreachable, // overlap check has not been implemented for this volume
    };
}

/// Returns a box that encompasses both a and b.
pub fn getEncompassingBox(a: anytype, b: anytype) Box2f {
    const box_a: Box2f = if (@TypeOf(a) == Box2f) a else a.getBoundingBox();
    const box_b: Box2f = if (@TypeOf(b) == Box2f) b else b.getBoundingBox();
    return .{
        .min = @min(box_a.min, box_b.min),
        .max = @max(box_a.max, box_b.max),
    };
}

fn checkOverlapBallBall(a: Ball2f, b: Ball2f) bool {
    const vec_diff = a.centre - b.centre;
    const r_sum = a.radius + b.radius;
    return calc.squaredSum(vec_diff) < r_sum * r_sum;
}

fn checkOverlapBoxBox(a: Box2f, b: Box2f) bool {
    const lo = @shuffle(f32, a.min, b.min, [4]i32{ 0, 1, -1, -2 }); // {a.min, b.min}
    const hi = @shuffle(f32, b.max, a.max, [4]i32{ 0, 1, -1, -2 }); // {b.max, a.max}
    return @reduce(.And, lo < hi);
}

fn checkOverlapBallBox(a: Ball2f, b: Box2f) bool {
    return calc.pointBoxDistSq(a.centre, b.min, b.max) < a.radius * a.radius;
}

fn checkOverlapLineBall(line: Line2f, ball: Ball2f) bool {
    const d_squared = calc.pointSegDistSq(ball.centre, line.start, line.end);
    return d_squared < ball.radius * ball.radius;
}

fn checkOverlapLineBox(line: Line2f, box: Box2f) bool {
    return calc.segmentIntersectsBox(line.start, line.end, box.min, box.max);
}

fn checkOverlapLineOrientedBox(line: Line2f, obb: OrientedBox2f) bool {
    const local_start = calc.transformToFrame(line.start, obb.centre, obb.axis);
    const local_end = calc.transformToFrame(line.end, obb.centre, obb.axis);
    return calc.segmentIntersectsBox(
        local_start,
        local_end,
        -obb.half_extents,
        obb.half_extents,
    );
}

fn checkOverlapOrientedBoxBall(obb: OrientedBox2f, ball: Ball2f) bool {
    const local = calc.transformToFrame(ball.centre, obb.centre, obb.axis);
    const d_squared = calc.pointBoxDistSq(local, -obb.half_extents, obb.half_extents);
    return d_squared < ball.radius * ball.radius;
}

fn checkOverlapOrientedBoxBox(obb: OrientedBox2f, box: Box2f) bool {
    const box_half = calc.scaledVec(0.5, box.max - box.min);
    return checkOverlapOrientedBoxes(
        obb.centre,
        obb.half_extents,
        obb.axis,
        box.getCentre(),
        box_half,
        Vec2f{ 1, 0 },
    );
}

fn checkOverlapOrientedBoxOrientedBox(a: OrientedBox2f, b: OrientedBox2f) bool {
    return checkOverlapOrientedBoxes(
        a.centre,
        a.half_extents,
        a.axis,
        b.centre,
        b.half_extents,
        b.axis,
    );
}

/// Separating-axis test shared by OrientedBox2f's overlap checks.
fn checkOverlapOrientedBoxes(
    c1: Vec2f,
    he1: Vec2f,
    ax1: Vec2f,
    c2: Vec2f,
    he2: Vec2f,
    ax2: Vec2f,
) bool {
    const ay1 = Vec2f{ -ax1[1], ax1[0] };
    const ay2 = Vec2f{ -ax2[1], ax2[0] };
    const d = c2 - c1;
    const axes = [4]Vec2f{ ax1, ay1, ax2, ay2 };
    for (axes) |axis| {
        const dist = @abs(calc.dotProduct(d, axis));
        const r1 = he1[0] * @abs(calc.dotProduct(ax1, axis)) + he1[1] * @abs(calc.dotProduct(ay1, axis));
        const r2 = he2[0] * @abs(calc.dotProduct(ax2, axis)) + he2[1] * @abs(calc.dotProduct(ay2, axis));
        if (dist > r1 + r2) return false;
    }
    return true;
}

pub const TestVolumes = struct {
    balls: std.ArrayList(Ball2f),
    boxes: std.ArrayList(Box2f),
    oriented_boxes: std.ArrayList(OrientedBox2f),
    const Self = @This();

    pub fn initRandom(
        allocator: std.mem.Allocator,
        random: std.Random,
        capacity: usize,
        size_dist: ProbDensityFunc,
        position_dist: ProbDensityFunc,
    ) !Self {
        var random_floats = try allocator.alloc(f32, capacity);
        defer allocator.free(random_floats);
        var random_vecs = try allocator.alloc(Vec2f, capacity);
        defer allocator.free(random_vecs);
        // generate random balls
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        var rand_balls = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            rand_balls.appendAssumeCapacity(.{
                .centre = random_vecs[i],
                .radius = random_floats[i],
            });
        }
        // generate random boxes
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        var rand_boxes = try std.ArrayList(Box2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const dim = Vec2f{ 2 * random_floats[i], 2 * random_floats[j] };
            const box = Box2f{ .min = random_vecs[i], .max = random_vecs[i] + dim };
            rand_boxes.appendAssumeCapacity(box);
        }
        // generate random oriented boxes
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        const tau_dist = ProbDensityFunc{ .uniform = .{ .min = 0, .max = math.tau } };
        var rand_obbs = try std.ArrayList(OrientedBox2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const angle = ProbDensityFunc.getFloat(tau_dist, random);
            rand_obbs.appendAssumeCapacity(.{
                .centre = random_vecs[i],
                .half_extents = .{ random_floats[i], random_floats[j] },
                .axis = .{ @cos(angle), @sin(angle) },
            });
        }
        return .{
            .balls = rand_balls,
            .boxes = rand_boxes,
            .oriented_boxes = rand_obbs,
        };
    }

    /// Loads test volumes from a csv file; rows must match below format:
    ///  - ball, centre_x, centre_y, radius
    ///  - box, min_x, min_y, max_x, max_y
    ///  - obb, centre_x, centre_y, half_extent_x, half_extent_y, axis_x, axis_y
    pub fn initCsv(allocator: std.mem.Allocator, io: std.Io, filepath: []const u8) !Self {
        const contents = try std.Io.Dir.cwd().readFileAlloc(io, filepath, allocator, .unlimited);
        defer allocator.free(contents);

        var ball_list: std.ArrayList(Ball2f) = .empty;
        errdefer ball_list.deinit(allocator);
        var box_list: std.ArrayList(Box2f) = .empty;
        errdefer box_list.deinit(allocator);
        var obb_list: std.ArrayList(OrientedBox2f) = .empty;
        errdefer obb_list.deinit(allocator);

        var lines = std.mem.tokenizeAny(u8, contents, "\r\n");
        while (lines.next()) |line| {
            const trimmed = std.mem.trim(u8, line, " \t");
            if (trimmed.len == 0) continue;
            var fields = std.mem.splitScalar(u8, trimmed, ',');
            const kind = fields.next() orelse return error.InvalidCsvRow;
            if (std.ascii.eqlIgnoreCase(kind, "ball")) {
                const cx = try parseCsvFloat(&fields);
                const cy = try parseCsvFloat(&fields);
                const r = try parseCsvFloat(&fields);
                try ball_list.append(allocator, .{ .centre = .{ cx, cy }, .radius = r });
            } else if (std.ascii.eqlIgnoreCase(kind, "box")) {
                const min_x = try parseCsvFloat(&fields);
                const min_y = try parseCsvFloat(&fields);
                const max_x = try parseCsvFloat(&fields);
                const max_y = try parseCsvFloat(&fields);
                try box_list.append(allocator, .{ .min = .{ min_x, min_y }, .max = .{ max_x, max_y } });
            } else if (std.ascii.eqlIgnoreCase(kind, "obb")) {
                const cx = try parseCsvFloat(&fields);
                const cy = try parseCsvFloat(&fields);
                const hx = try parseCsvFloat(&fields);
                const hy = try parseCsvFloat(&fields);
                const ax = try parseCsvFloat(&fields);
                const ay = try parseCsvFloat(&fields);
                try obb_list.append(allocator, .{
                    .centre = .{ cx, cy },
                    .half_extents = .{ hx, hy },
                    .axis = .{ ax, ay },
                });
            } else {
                return error.UnknownVolumeType;
            }
        }
        ball_list.shrinkAndFree(allocator, ball_list.items.len);
        box_list.shrinkAndFree(allocator, box_list.items.len);
        obb_list.shrinkAndFree(allocator, obb_list.items.len);

        return .{
            .balls = ball_list,
            .boxes = box_list,
            .oriented_boxes = obb_list,
        };
    }

    pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
        self.balls.deinit(allocator);
        self.boxes.deinit(allocator);
        self.oriented_boxes.deinit(allocator);
    }

    pub fn getRandomBodies(self: *Self, comptime T: type) []T {
        return switch (T) {
            Ball2f => self.balls.items,
            Box2f => self.boxes.items,
            OrientedBox2f => self.oriented_boxes.items,
            else => unreachable,
        };
    }

    fn parseCsvFloat(fields: *std.mem.SplitIterator(u8, .scalar)) !f32 {
        const field = fields.next() orelse return error.InvalidCsvRow;
        return std.fmt.parseFloat(f32, field);
    }
};

const testing = std.testing;
const test_num_volumes = 16000;
const test_min_extent = 0.1;
const test_max_extent = 5.0;
const test_space_min = -50;
const test_space_max = 50;

test "balls overlap" {
    const a = Ball2f{ .centre = .{ 0, 0 }, .radius = 1.0 };
    const b1 = Ball2f{ .centre = .{ 0.5, 0.5 }, .radius = 0.1 };
    const b2 = Ball2f{ .centre = .{ 1.5, 0.0 }, .radius = 0.6 };
    const b3 = Ball2f{ .centre = .{ 1.0, 1.0 }, .radius = 0.4 };
    const b4 = Ball2f{ .centre = .{ 1.5, 0.0 }, .radius = 0.5 };
    const check_1 = checkOverlapBallBall(a, b1);
    try testing.expectEqual(true, check_1);
    const check_2 = checkOverlapBallBall(a, b2);
    try testing.expectEqual(true, check_2);
    const check_3 = checkOverlapBallBall(a, b3);
    try testing.expectEqual(false, check_3);
    const check_4 = checkOverlapBallBall(a, b4);
    try testing.expectEqual(false, check_4);
}

test "boxes overlap" {
    const a = Box2f{ .min = .{ 0.0, 0.0 }, .max = .{ 1.0, 1.0 } };
    const b1 = Box2f{ .min = .{ 0.25, 0.25 }, .max = .{ 1.25, 1.25 } };
    const b2 = Box2f{ .min = .{ 0.5, 0.5 }, .max = .{ 0.75, 0.75 } };
    const b3 = Box2f{ .min = .{ 0.75, 0.75 }, .max = .{ 1.5, 1.5 } };
    const b4 = Box2f{ .min = .{ 1.5, 1.5 }, .max = .{ 1.75, 1.75 } };
    const check_1 = checkOverlapBoxBox(a, b1);
    const check_2 = checkOverlapBoxBox(a, b2);
    const check_3 = checkOverlapBoxBox(a, b3);
    const check_4 = checkOverlapBoxBox(a, b4);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(true, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
}

test "ball-box overlap" {
    const a = Ball2f{ .centre = .{ 0.0, 0.0 }, .radius = 3 };
    const b1 = Box2f{ .min = .{ -1.0, -0.5 }, .max = .{ 1.0, 0.5 } };
    const b2 = Box2f{ .min = .{ -3.0, 4.5 }, .max = .{ 3.0, 5.5 } };
    const b3 = Box2f{ .min = .{ -0.5, -8.0 }, .max = .{ 0.5, -2.0 } };
    const b4 = Box2f{ .min = .{ -4.5, -4.5 }, .max = .{ -3.5, -3.5 } };
    const b5 = Box2f{ .min = .{ 1.5, 1.0 }, .max = .{ 2.5, 7.0 } };
    const check_1 = checkVolumesOverlap(a, b1);
    const check_2 = checkVolumesOverlap(a, b2);
    const check_3 = checkVolumesOverlap(a, b3);
    const check_4 = checkVolumesOverlap(a, b4);
    const check_5 = checkVolumesOverlap(a, b5);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(false, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
    try testing.expectEqual(true, check_5);
}

test "oriented box - ball overlap" {
    const obb = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 2, 1 },
        .axis = .{ 0, 1 },
    };
    const b1 = Ball2f{ .centre = .{ 0.5, 1.5 }, .radius = 0.6 };
    const b2 = Ball2f{ .centre = .{ 5, 5 }, .radius = 0.5 };
    const b3 = Ball2f{ .centre = .{ 1.5, 0 }, .radius = 0.6 };
    const b4 = Ball2f{ .centre = .{ 1.5, 0 }, .radius = 0.4 };
    try testing.expectEqual(true, checkVolumesOverlap(obb, b1));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b2));
    try testing.expectEqual(true, checkVolumesOverlap(obb, b3));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b4));
}

test "oriented box - box overlap" {
    const obb = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 3, 1 },
        .axis = .{ 0, 1 },
    };
    const b1 = Box2f{ .min = .{ -0.5, -0.5 }, .max = .{ 0.5, 0.5 } };
    const b2 = Box2f{ .min = .{ 2, 2 }, .max = .{ 3, 3 } };
    const b3 = Box2f{ .min = .{ -10, -10 }, .max = .{ -9, -9 } };
    const b4 = Box2f{ .min = .{ 0.8, 2.8 }, .max = .{ 1.5, 3.5 } };
    try testing.expectEqual(true, checkVolumesOverlap(obb, b1));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b2));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b3));
    try testing.expectEqual(true, checkVolumesOverlap(obb, b4));
    // argument order shouldn't matter
    try testing.expectEqual(true, checkVolumesOverlap(b1, obb));
    try testing.expectEqual(false, checkVolumesOverlap(b2, obb));
}

test "oriented box - oriented box overlap" {
    const a = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 1, 1 },
        .axis = .{ math.sqrt1_2, math.sqrt1_2 },
    };
    const b1 = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 0.2, 0.2 },
        .axis = .{ 1, 0 },
    };
    const b2 = OrientedBox2f{
        .centre = .{ 1.15, 1.15 },
        .half_extents = .{ 0.15, 0.15 },
        .axis = .{ 1, 0 },
    };
    const b3 = OrientedBox2f{
        .centre = .{ 5, 5 },
        .half_extents = .{ 1, 1 },
        .axis = .{ 0, 1 },
    };
    try testing.expectEqual(true, checkVolumesOverlap(a, b1));
    try testing.expectEqual(false, checkVolumesOverlap(a, b2));
    try testing.expectEqual(false, checkVolumesOverlap(a, b3));
    try testing.expectEqual(true, checkVolumesOverlap(b1, a));
}

test "unrotated oriented box matches its axis-aligned box" {
    const centre = Vec2f{ 1.0, -0.5 };
    const half = Vec2f{ 0.7, 1.3 };
    const obb = OrientedBox2f{ .centre = centre, .half_extents = half, .axis = .{ 1, 0 } };
    const equivalent_box = Box2f{ .min = centre - half, .max = centre + half };
    var rng = std.Random.Xoshiro256.init(2);
    var test_vols = try TestVolumes.initRandom(
        testing.allocator,
        rng.random(),
        test_num_volumes,
        .{ .uniform = .{ .min = test_min_extent, .max = test_max_extent } },
        .{ .uniform = .{ .min = test_space_min, .max = test_space_max } },
    );
    defer test_vols.deinit(testing.allocator);

    for (test_vols.getRandomBodies(Ball2f)) |b| {
        const expected = checkVolumesOverlap(equivalent_box, b);
        try testing.expectEqual(expected, checkVolumesOverlap(obb, b));
        try testing.expectEqual(expected, checkVolumesOverlap(b, obb));
    }
    for (test_vols.getRandomBodies(Box2f)) |b| {
        const expected = checkVolumesOverlap(equivalent_box, b);
        try testing.expectEqual(expected, checkVolumesOverlap(obb, b));
        try testing.expectEqual(expected, checkVolumesOverlap(b, obb));
    }
}

test "line-ball overlap" {
    const line = Line2f{ .start = .{ 0, 0 }, .end = .{ 4, 0 } };
    const b1 = Ball2f{ .centre = .{ 2, 0.3 }, .radius = 0.4 };
    const b2 = Ball2f{ .centre = .{ 2, 3.0 }, .radius = 0.4 };
    const b3 = Ball2f{ .centre = .{ -0.1, 0 }, .radius = 0.15 };
    const b4 = Ball2f{ .centre = .{ 4.1, 0 }, .radius = 0.15 };
    const b5 = Ball2f{ .centre = .{ 5, 0 }, .radius = 0.5 };
    try testing.expectEqual(true, checkVolumesOverlap(line, b1));
    try testing.expectEqual(false, checkVolumesOverlap(line, b2));
    try testing.expectEqual(true, checkVolumesOverlap(line, b3));
    try testing.expectEqual(true, checkVolumesOverlap(line, b4));
    try testing.expectEqual(false, checkVolumesOverlap(line, b5));
    try testing.expectEqual(true, checkVolumesOverlap(b1, line));
    try testing.expectEqual(false, checkVolumesOverlap(b5, line));
}

test "line - oriented box overlap" {
    const obb = OrientedBox2f{ .centre = .{ 0, 0 }, .half_extents = .{ 3, 1 }, .axis = .{ 0, 1 } };
    const through = Line2f{ .start = .{ -2, 0 }, .end = .{ 2, 0 } };
    const miss_far = Line2f{ .start = .{ 2, 2 }, .end = .{ 3, 3 } };
    const miss_rotated = Line2f{ .start = .{ 1.5, 0 }, .end = .{ 1.5, 5 } };
    const inside = Line2f{ .start = .{ 0.9, 2.9 }, .end = .{ 1.2, 3.2 } };
    try testing.expectEqual(true, checkVolumesOverlap(through, obb));
    try testing.expectEqual(false, checkVolumesOverlap(miss_far, obb));
    try testing.expectEqual(false, checkVolumesOverlap(miss_rotated, obb));
    try testing.expectEqual(true, checkVolumesOverlap(inside, obb));
    try testing.expectEqual(true, checkVolumesOverlap(obb, through));
    try testing.expectEqual(false, checkVolumesOverlap(obb, miss_rotated));
}

test "line-box overlap" {
    const line = Line2f{ .start = .{ 0, 0 }, .end = .{ 4, 0 } };
    const b1 = Box2f{ .min = .{ 1, -0.2 }, .max = .{ 3, 0.2 } };
    const b2 = Box2f{ .min = .{ 1, 2 }, .max = .{ 3, 3 } };
    const b3 = Box2f{ .min = .{ -3, -3 }, .max = .{ -2, -2 } };
    const b4 = Box2f{ .min = .{ -0.5, -0.5 }, .max = .{ 4.5, 0.5 } };
    const b5 = Box2f{ .min = .{ 4, -1 }, .max = .{ 5, 1 } };
    try testing.expectEqual(true, checkVolumesOverlap(line, b1));
    try testing.expectEqual(false, checkVolumesOverlap(line, b2));
    try testing.expectEqual(false, checkVolumesOverlap(line, b3));
    try testing.expectEqual(true, checkVolumesOverlap(line, b4));
    try testing.expectEqual(true, checkVolumesOverlap(line, b5));
    try testing.expectEqual(true, checkVolumesOverlap(b1, line));
    try testing.expectEqual(false, checkVolumesOverlap(b2, line));
}

test "axis-aligned and perpendicuar lines vs box" {
    const box = Box2f{ .min = .{ 0, 0 }, .max = .{ 2, 2 } };
    const vertical_hit = Line2f{ .start = .{ 1, -5 }, .end = .{ 1, 5 } };
    const vertical_miss = Line2f{ .start = .{ 5, -5 }, .end = .{ 5, 5 } };
    const point_inside = Line2f{ .start = .{ 1, 1 }, .end = .{ 1, 1 } };
    const point_outside = Line2f{ .start = .{ 5, 5 }, .end = .{ 5, 5 } };
    try testing.expectEqual(true, checkVolumesOverlap(vertical_hit, box));
    try testing.expectEqual(false, checkVolumesOverlap(vertical_miss, box));
    try testing.expectEqual(true, checkVolumesOverlap(point_inside, box));
    try testing.expectEqual(false, checkVolumesOverlap(point_outside, box));
}

test "encompassing boxes" {
    const a = Box2f{ .min = .{ -0.139, -0.139 }, .max = .{ 0.139, 0.139 } };
    const b = Box2f{ .min = .{ -0.735, -0.2 }, .max = .{ 0.2, 0.735 } };
    const c = getEncompassingBox(a, b);
    try testing.expectEqual(@min(a.min, b.min), c.min);
    try testing.expectEqual(@max(a.max, b.max), c.max);
}

test "empty ball behaves as expected" {
    var rng = std.Random.Xoshiro256.init(0);
    var test_vols = try TestVolumes.initRandom(
        testing.allocator,
        rng.random(),
        test_num_volumes,
        .{ .uniform = .{ .min = test_min_extent, .max = test_max_extent } },
        .{ .uniform = .{ .min = test_space_min, .max = test_space_max } },
    );
    defer test_vols.deinit(testing.allocator);
    const balls = test_vols.getRandomBodies(Ball2f);

    for (balls) |b| {
        try testing.expectEqual(false, checkVolumesOverlap(empty_ball, b));
        try testing.expectEqual(false, checkVolumesOverlap(b, empty_ball));
    }
}

test "empty box behaves as expected" {
    var rng = std.Random.Xoshiro256.init(1);
    var test_vols = try TestVolumes.initRandom(
        testing.allocator,
        rng.random(),
        test_num_volumes,
        .{ .uniform = .{ .min = test_min_extent, .max = test_max_extent } },
        .{ .uniform = .{ .min = test_space_min, .max = test_space_max } },
    );
    defer test_vols.deinit(testing.allocator);
    const boxes = test_vols.getRandomBodies(Box2f);

    for (boxes) |b| {
        try testing.expectEqual(b, getEncompassingBox(empty_ball, b));
        try testing.expectEqual(b, getEncompassingBox(b, empty_ball));
        try testing.expectEqual(false, checkVolumesOverlap(empty_ball, b));
        try testing.expectEqual(false, checkVolumesOverlap(b, empty_ball));
    }
}
