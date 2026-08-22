const std = @import("std");
const builtin = @import("builtin");
const zgrd = @import("zgrd");
const calc = zgrd.calc;
const index = zgrd.index;
const st = zgrd.square_tree;
const vol = zgrd.volume;
const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const Line2f = vol.Line2f;
const OrientedBox2f = vol.OrientedBox2f;
const timer = std.Io.Clock.awake;
const max_capacity = 200_000;
const min_trials = 5;
const min_num_vols = 100;
const usage_msg =
    \\Usage: zgrd-bench [options]
    \\  -i: Set an input file (csv) to load test volumes from (see readme for correct format)
    \\  -p: String defining pdf used to generate test volume positions; default "N(5,2.5)"
    \\      Parameters in parentheses for (N)ormal: (mean,std_dev).
    \\      Parameters in parentheses for (U)niform: (min,max).
    \\  -s: String for pdf used to generate test volumes sizes; default "U(0.001,0.05)"
    \\  -t: Number of times to repeat each benchmark (for stable timing averages); default = 30
    \\  -v: Number of volumes to generate (ignored if using -i); default = 20000 (2000 in debug)
    \\
;
var input_file: ?[]const u8 = null;
var output_dir: ?[]const u8 = null;
var random_vols: vol.TestVolumes = undefined;
var position_dist: calc.ProbDensityFunc = .{ .normal = .{ .mean = 5.0, .stddev = 1.5 } };
var size_dist: calc.ProbDensityFunc = .{ .uniform = .{ .min = 0.001, .max = 0.05 } };
var num_trials: u8 = 30;
var num_vols: u24 = if (builtin.mode == .ReleaseFast) 20_000 else 2_000;

/// Fetches the value following a flag, erroring with the usage message if none was supplied.
fn nextArgValue(args_iter: *std.process.Args.Iterator, flag: []const u8) ![:0]const u8 {
    return args_iter.next() orelse {
        std.debug.print("Missing value for '{s}'.\n{s}", .{ flag, usage_msg });
        return error.MissingArgumentValue;
    };
}

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    var args_iter = try std.process.Args.Iterator.initAllocator(init.minimal.args, allocator);
    defer args_iter.deinit();
    _ = args_iter.next(); // skip the program name

    while (args_iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "-i")) {
            const file = try nextArgValue(&args_iter, arg);
            const cwd = std.Io.Dir.cwd();
            cwd.access(init.io, file, .{ .read = true }) catch |err| {
                std.debug.print("Unable to read from {s}\n", .{file});
                return err;
            };
            input_file = file;
        } else if (std.mem.eql(u8, arg, "-p")) {
            const pdf_str = try nextArgValue(&args_iter, arg);
            position_dist = calc.ProbDensityFunc.fromPdfString(pdf_str) catch {
                std.debug.print("Could not parse pdf string '{s}':\n{s}", .{ pdf_str, usage_msg });
                return error.InvalidPdfArgument;
            };
        } else if (std.mem.eql(u8, arg, "-s")) {
            const pdf_str = try nextArgValue(&args_iter, arg);
            size_dist = calc.ProbDensityFunc.fromPdfString(pdf_str) catch {
                std.debug.print("Could not parse pdf string '{s}':\n{s}", .{ pdf_str, usage_msg });
                return error.InvalidPdfArgument;
            };
        } else if (std.mem.eql(u8, arg, "-t")) {
            const val_str = try nextArgValue(&args_iter, arg);
            num_trials = std.fmt.parseInt(u8, val_str, 10) catch {
                std.debug.print("Could not parse trial count '{s}':\n{s}", .{ val_str, usage_msg });
                return error.InvalidTrialCount;
            };
        } else if (std.mem.eql(u8, arg, "-v")) {
            const val_str = try nextArgValue(&args_iter, arg);
            num_vols = std.fmt.parseInt(u24, val_str, 10) catch {
                std.debug.print("Could not parse volume count '{s}':\n{s}", .{ val_str, usage_msg });
                return error.InvalidVolumeCount;
            };
        } else {
            std.debug.print("Unrecognised argument '{s}'.\n{s}", .{ arg, usage_msg });
            return error.UnrecognisedArgument;
        }
    }
    if (num_trials < min_trials) {
        std.debug.print(
            "Requested {} trials is below the minimum required ({}).\n{s}",
            .{ num_trials, min_trials, usage_msg },
        );
        return error.TooFewTrials;
    }

    if (input_file) |file| {
        random_vols = try vol.TestVolumes.initCsv(allocator, init.io, file);
        std.debug.print("Running index/overlap micro-benchmarks using volumes loaded from '{s}'...\n", .{file});
    } else {
        if (min_num_vols > num_vols or num_vols > max_capacity) {
            std.debug.print(
                "Requested {} volumes outside supported range ({} - {}).\n{s}",
                .{ num_vols, min_num_vols, max_capacity, usage_msg },
            );
            return error.InvalidNumberVolumes;
        }

        var prng = std.Random.DefaultPrng.init(0);
        random_vols = try vol.TestVolumes.initRandom(
            allocator,
            prng.random(),
            num_vols,
            size_dist,
            position_dist,
        );
        std.debug.print("Running index/overlap micro-benchmarks for {} vols...\n", .{num_vols});
    }
    defer random_vols.deinit(allocator);

    try benchmarkIndexing(allocator, init.io);
    benchmarkOverlapChecks(init.io);
    try benchmarkSquareTrees(allocator, init.io);
}

fn benchmarkSquareTrees(allocator: std.mem.Allocator, io: std.Io) !void {
    inline for (.{ Ball2f, Box2f }) |V| {
        std.debug.print(
            "\nRunning tree benchmarks for {d} {any}...\n",
            .{ random_vols.getRandomBodies(V).len, V },
        );
        std.debug.print(
            "\n| Indexer type     | size  | overlaps | max leafs | add (ns) | update (ns) | overlap (ns) | tick (ms) |\n",
            .{},
        );
        const indexer_types = .{
            index.Indexer2f(2, 1, 3, .Morton),
            index.Indexer2f(2, 1, 4, .Morton),
            index.Indexer2f(2, 1, 5, .Morton),
            index.Indexer2f(2, 3, 3, .Morton),
            index.Indexer2f(2, 6, 0, .Morton),
            index.Indexer2f(2, 1, 6, .Morton),
            index.Indexer2f(2, 4, 3, .Morton),
            index.Indexer2f(2, 1, 7, .Morton),
            index.Indexer2f(2, 1, 8, .Morton),
            index.Indexer2f(4, 1, 1, .Morton),
            index.Indexer2f(4, 1, 2, .Morton),
            index.Indexer2f(4, 3, 0, .Morton),
            index.Indexer2f(4, 1, 3, .Morton),
            index.Indexer2f(4, 4, 0, .Morton),
            index.Indexer2f(4, 1, 4, .Morton),
            index.Indexer2f(4, 1, 1, .Zigzag),
            index.Indexer2f(4, 1, 2, .Zigzag),
            index.Indexer2f(4, 2, 1, .Zigzag),
            index.Indexer2f(4, 3, 0, .Zigzag),
            index.Indexer2f(4, 1, 3, .Zigzag),
            index.Indexer2f(4, 2, 2, .Zigzag),
            index.Indexer2f(4, 1, 4, .Zigzag),
        };
        inline for (indexer_types) |Indexer| {
            const bench_results = try benchMarkTree(st.SquareTree(Indexer, V, u32), V, allocator, io);
            printBenchmarkRow(indexerLabel(Indexer), bench_results);
        }
    }
}

/// Names an indexer as "<curve> <cells per axis> x <effective depth> [<levels>]".
fn indexerLabel(comptime Indexer: type) []const u8 {
    return std.fmt.comptimePrint("{s} {d} x {d} [{d}]", .{
        @tagName(Indexer.curve),
        Indexer.base,
        Indexer.effective_depth,
        Indexer.depth,
    });
}

/// Prints one benchmark result as a row matching the table header printed in main.
fn printBenchmarkRow(name: []const u8, stats: BenchmarkStats) void {
    std.debug.print(
        "| {s:<16} | {d:>3} B | {d:>8} |  {d:>5}  | {d:>8.3} | {d:>11.3} | {d:>12.3} | {d:>9.3} |\n",
        .{
            name,
            stats.struct_size,
            stats.overlaps,
            stats.max_leaf,
            stats.ns_add,
            stats.ns_update,
            stats.ns_overlap,
            stats.ms_tick,
        },
    );
}

fn elapsedNs(t1: std.Io.Timestamp, t2: std.Io.Timestamp) f64 {
    return @floatFromInt(std.Io.Timestamp.durationTo(t1, t2).toNanoseconds());
}

// TODO: format nicely and add average at bottom of table
fn benchmarkIndexing(allocator: std.mem.Allocator, io: std.Io) !void {
    const IndexerTypes = [_]type{
        index.Indexer2f(2, 1, 2, index.Curve.Morton),
        index.Indexer2f(2, 1, 3, index.Curve.Morton),
        index.Indexer2f(2, 1, 4, index.Curve.Morton),
        index.Indexer2f(2, 1, 5, index.Curve.Morton),
        index.Indexer2f(2, 1, 6, index.Curve.Morton),
        index.Indexer2f(2, 1, 7, index.Curve.Morton),
        index.Indexer2f(4, 1, 1, index.Curve.Morton),
        index.Indexer2f(4, 1, 2, index.Curve.Spring),
        index.Indexer2f(4, 1, 3, index.Curve.Zigzag),
        index.Indexer2f(4, 1, 4, index.Curve.Zigzag),
    };
    inline for (IndexerTypes) |Indexer| {
        const pt1 = random_vols.balls.items[0].centre;
        const pt2 = random_vols.balls.items[1].centre;
        var indexer = try Indexer.init(pt1, pt2);
        var indexes = try allocator.alloc(Indexer.CurveIndex, random_vols.boxes.items.len);
        defer allocator.free(indexes);
        var index_sum: f64 = 0;

        const t_0 = timer.now(io);
        for (random_vols.boxes.items, 0..) |b, i| {
            const index_of_b = indexer.getLeafIndexForPoint(b.getCentre());
            index_sum += @as(f64, @floatFromInt(index_of_b));
            indexes[i] = index_of_b;
        }
        const t_1 = timer.now(io);

        // compute a sum of the inter-leaf distances
        var index_dist_sum: f64 = 0;
        var last_centre = indexer.getLeafCellBoundary(0).getCentre();
        for (1..Indexer.num_leaves) |i| {
            const centre_i = indexer.getLeafCellBoundary(@truncate(i)).getCentre();
            index_dist_sum += calc.norm(centre_i - last_centre);
            last_centre = centre_i;
        }
        const avg_inter_leaf_dist = index_dist_sum / @as(f64, @floatFromInt(Indexer.num_leaves - 1));
        const avg_index_time = elapsedNs(t_0, t_1) / @as(f64, @floatFromInt(random_vols.boxes.items.len));
        std.debug.print(
            "{s}: point indexing took {d:.3} ns/point; avg_inter_leaf_dist = {d:.4}\n",
            .{ indexerLabel(Indexer), avg_index_time, avg_inter_leaf_dist },
        );
    }
}

// TODO: format nicely and add average at bottom of table
fn benchmarkOverlapChecks(io: std.Io) void {
    const trials: usize = num_trials;
    const half_trials = (trials + 1) / 2; // rounds up, so a trial count of 1 still runs the mixed check once
    var overlap_count_1: u32 = 0;
    var overlap_count_2: u32 = 0;
    var overlap_count_3: u32 = 0;
    var overlap_count_4: u32 = 0;
    var overlap_count_5: u32 = 0;

    const t_0 = timer.now(io);
    var first_ball = random_vols.balls.items[0];
    first_ball.radius = first_ball.radius * 25;
    for (0..trials) |_| {
        for (random_vols.balls.items) |b| {
            const overlap = vol.checkVolumesOverlap(first_ball, b);
            overlap_count_1 += if (overlap) 1 else 0;
        }
    }
    const t_1 = timer.now(io);

    const first_box = random_vols.boxes.items[0];
    const query_box = first_box.getScaled(25);
    for (0..trials) |_| {
        for (random_vols.boxes.items) |b| {
            const overlap = vol.checkVolumesOverlap(query_box, b);
            overlap_count_2 += if (overlap) 1 else 0;
        }
    }
    const t_2 = timer.now(io);

    for (0..half_trials) |_| {
        for (random_vols.balls.items) |b| {
            const overlap = vol.checkVolumesOverlap(query_box, b);
            overlap_count_3 += if (overlap) 1 else 0;
        }
        for (random_vols.boxes.items) |b| {
            const overlap = vol.checkVolumesOverlap(first_ball, b);
            overlap_count_3 += if (overlap) 1 else 0;
        }
    }
    const t_3 = timer.now(io);

    const query_obb = random_vols.oriented_boxes.items[0].getScaled(25);
    for (0..trials) |_| {
        for (random_vols.boxes.items) |b| {
            const overlap = vol.checkVolumesOverlap(query_obb, b);
            overlap_count_4 += if (overlap) 1 else 0;
        }
    }
    const t_4 = timer.now(io);

    const query_line = Line2f{ .start = first_box.min, .end = first_box.max };
    for (0..trials) |_| {
        for (random_vols.boxes.items) |b| {
            const overlap = vol.checkVolumesOverlap(query_line, b);
            overlap_count_5 += if (overlap) 1 else 0;
        }
    }
    const t_5 = timer.now(io);

    std.debug.print(
        "Overlaps benchmark: found {}/{}/{}/{}/{} overlaps.\n",
        .{ overlap_count_1, overlap_count_2, overlap_count_3, overlap_count_4, overlap_count_5 },
    );

    const ball_checks: f64 = @floatFromInt(trials * random_vols.balls.items.len);
    const box_checks: f64 = @floatFromInt(trials * random_vols.boxes.items.len);
    const mixed_checks: f64 = @floatFromInt(
        half_trials * (random_vols.balls.items.len + random_vols.boxes.items.len),
    );
    const obb_box_checks: f64 = @floatFromInt(trials * random_vols.boxes.items.len);
    const line_box_checks: f64 = @floatFromInt(trials * random_vols.boxes.items.len);
    std.debug.print(
        "Average check time (ns/op): balls {d:.3}; boxes {d:.3}; ball-box {d:.3}; obb-box {d:.3}; line-box {d:.3}.\n",
        .{
            elapsedNs(t_0, t_1) / ball_checks,
            elapsedNs(t_1, t_2) / box_checks,
            elapsedNs(t_2, t_3) / mixed_checks,
            elapsedNs(t_3, t_4) / obb_box_checks,
            elapsedNs(t_4, t_5) / line_box_checks,
        },
    );
}

// TODO: add average at bottom of table
/// Average timings for one grid configuration + a simulated frame's cost
const BenchmarkStats = struct {
    ns_add: f64,
    ns_update: f64,
    ns_overlap: f64,
    ms_tick: f64,
    max_leaf: u16,
    overlaps: usize,
    struct_size: usize,
};

fn benchMarkTree(
    comptime TreeType: type,
    comptime VolumeType: type,
    allocator: std.mem.Allocator,
    io: std.Io,
) !BenchmarkStats {
    var tree = try TreeType.init(allocator, Vec2f{ 0, 0 }, Vec2f{ 10, 10 }, max_capacity);
    defer tree.deinit(allocator);
    const bodies = random_vols.getRandomBodies(VolumeType);
    const overlap_buff = try allocator.alloc(TreeType.OverlapPair, 1024 * bodies.len);
    var entity_indexes = try allocator.alloc(TreeType.ClientIdType, bodies.len);
    var overlaps: usize = 0;
    var ns_add: f64 = 0;
    var ns_update: f64 = 0;
    var ns_query_idx: f64 = 0;

    for (0..num_trials) |_| {
        const t_0 = timer.now(io);

        tree.clearStoredVolumes();
        for (0..entity_indexes.len) |i| entity_indexes[i] = @intCast(i);
        try tree.addVolumes(bodies, entity_indexes);
        const t_1 = timer.now(io);

        tree.updateBounds();
        const t_2 = timer.now(io);

        overlaps += (try tree.findSelfOverlaps(overlap_buff)).len;
        const t_3 = timer.now(io);

        ns_add += elapsedNs(t_0, t_1);
        ns_update += elapsedNs(t_1, t_2);
        ns_query_idx += elapsedNs(t_2, t_3);
    }

    const ns_total = (ns_add + ns_update + ns_query_idx);
    const checks: f64 = @floatFromInt(bodies.len * num_trials);
    return .{
        .ns_add = ns_add / checks,
        .ns_update = ns_update / checks,
        .ns_overlap = ns_query_idx / checks,
        .ms_tick = ns_total / @as(f64, @floatFromInt(@as(u32, num_trials) * 1_000_000)),
        .max_leaf = tree.maxLeafOccupancy(),
        .overlaps = overlaps / num_trials,
        .struct_size = @sizeOf(TreeType),
    };
}
