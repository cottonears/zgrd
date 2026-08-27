const std = @import("std");

/// Wraps a slice of caller-owned memory, tracking how much of it is filled.
/// Holds mutable cursor state: always pass/store by pointer (`*BoundedList(T)`).
pub fn BoundedList(comptime T: type) type {
    return struct {
        capacity: usize,
        index: usize = 0,
        items: []T = undefined,
        const Self = @This();

        /// Inits an empty list backed by a slice of caller-owned memory.
        pub fn init(slice: []T) Self {
            return .{
                .items = slice,
                .capacity = slice.len,
            };
        }

        /// Appends an item; returns BufferCapacityExceeded if at capacity.
        pub fn add(self: *Self, item: T) !void {
            if (self.index >= self.capacity) return error.BufferCapacityExceeded;
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
    };
}

test "add to bounded list" {
    var my_array: [64]usize = undefined;
    var bound_list = BoundedList(usize).init(&my_array);
    for (0..32) |i| try bound_list.add(i);
    bound_list.clear();
}
