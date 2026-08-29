//! This module is for general-purpose data structures used around the project.
const std = @import("std");

/// Wraps a slice of caller-owned memory, tracking how much of it is filled.
/// Holds mutable cursor state: always pass/store by pointer (`*BoundedList(T)`).
pub fn BoundedList(comptime T: type) type {
    return struct {
        index: usize = 0,
        items: []T = undefined,
        const Self = @This();

        /// Inits an empty list backed by a slice of caller-owned memory.
        pub fn init(slice: []T) Self {
            return .{ .items = slice };
        }

        /// Appends an item; returns BufferCapacityExceeded if at capacity.
        pub fn add(self: *Self, item: T) !void {
            if (self.index >= self.items.len) return error.BufferCapacityExceeded;
            self.items[self.index] = item;
            self.index += 1;
        }

        /// Empties the list without releasing its backing memory.
        pub fn clear(self: *Self) void {
            self.index = 0;
        }

        /// Gets a slice containing the current items.
        pub fn getItems(self: *const Self) []T {
            return self.items[0..self.index];
        }

        pub fn sortAsc(self: *Self) void {
            std.sort.pdq(T, self.items[0..self.index], {}, asc);
        }

        pub fn sortDesc(self: *Self) void {
            std.sort.pdq(T, self.items[0..self.index], {}, desc);
        }

        fn asc(_: void, a: T, b: T) bool {
            return a < b;
        }
        fn desc(_: void, a: T, b: T) bool {
            return a > b;
        }
    };
}

pub fn DataTable(
    comptime T: type,
    comptime num_cols: u8,
    comptime headers: [num_cols][]const u8,
    comptime formats: [num_cols][]const u8,
) type {
    return struct {
        column_data: [num_cols]BoundedList(T) = undefined,
        num_rows: usize = 0,
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, capacity: usize) !Self {
            var cols: [num_cols]BoundedList(T) = undefined;
            var cols_created: usize = 0;
            errdefer for (0..cols_created) |i| allocator.free(cols[i].items);
            for (0..num_cols) |i| {
                const col_slice = try allocator.alloc(T, capacity);
                cols[i] = BoundedList(T).init(col_slice);
                cols_created += 1;
            }
            return .{ .column_data = cols };
        }

        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            for (0..num_cols) |i| allocator.free(self.column_data[i].items);
        }

        pub fn addRow(self: *Self, vals: [num_cols]T) !void {
            for (0..num_cols) |j| try self.column_data[j].add(vals[j]);
            self.num_rows += 1;
        }

        /// Sorts column data and gets range + IQR stats for each: { min, q1, q2, q3, max }.
        /// Doesn't interpolate between indexes: inacurate for a low sample sizes.
        pub fn computeStats(self: *Self) ?[num_cols][5]T {
            if (self.column_data[0].index == 0) return null;
            var col_stats: [num_cols][5]T = undefined;
            for (0..num_cols) |j| {
                self.column_data[j].sortAsc();
                const items = self.column_data[j].getItems();
                const min = items[0];
                const q1 = items[1 * items.len / 4];
                const q2 = items[2 * items.len / 4];
                const q3 = items[3 * items.len / 4];
                const max = items[items.len - 1];
                col_stats[j] = .{ min, q1, q2, q3, max };
            }
            return col_stats;
        }

        /// Builds a multi-line string representing a table's stats.
        pub fn getStatsTable(self: *Self, allocator: std.mem.Allocator) !std.ArrayList(u8) {
            var string = try std.ArrayList(u8).initCapacity(allocator, num_cols * 64);
            errdefer string.deinit(allocator);
            const col_stats = self.computeStats() orelse return string;
            try string.appendSlice(allocator, "|     |");
            for (0..num_cols) |j| {
                try string.appendSlice(allocator, headers[j]);
                try string.append(allocator, '|');
            }
            const row_titles: [5][]const u8 = .{ " min ", " q1  ", " q2  ", " q3  ", " max " };
            var stat_buff: [32]u8 = undefined;
            for (0..5) |i| {
                try string.appendSlice(allocator, "\n|");
                try string.appendSlice(allocator, row_titles[i]);
                try string.append(allocator, '|');
                inline for (0..num_cols) |j| {
                    const cell_val = col_stats[j][i];
                    const stat_str = try std.fmt.bufPrint(&stat_buff, formats[j], .{cell_val});
                    try string.appendSlice(allocator, stat_str);
                    try string.append(allocator, '|');
                }
            }
            try string.append(allocator, '\n');
            return string;
        }
    };
}

const testing = std.testing;
const test_alloc = testing.allocator;

test "add to data table" {
    const col_headers: [3][]const u8 = .{ " count ", " pressure   ", " temperature " };
    const col_formats: [3][]const u8 = .{ " {d:>5.0} ", " {d:>6.1} kPa ", " {d:>9.2}°K " };
    var my_table = try DataTable(f64, 3, col_headers, col_formats).init(test_alloc, 100);
    defer my_table.deinit(test_alloc);
    const n = 42.0;
    for (0..100) |i| {
        const t = 260 + @as(f64, @floatFromInt(i));
        const p = 8.314 * 42.29 * t / 1000.0;
        try my_table.addRow(.{ n, p, t });
    }
    var table_str = try my_table.getStatsTable(test_alloc);
    defer table_str.deinit(test_alloc);
    std.debug.print("{s}", .{table_str.items});
}
