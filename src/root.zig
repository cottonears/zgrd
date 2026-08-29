const std = @import("std");
pub const calc = @import("calc.zig");
pub const data = @import("data.zig");
pub const index = @import("index.zig");
pub const square_tree = @import("square_tree.zig");
pub const volume = @import("volume.zig");
pub const svg = @import("svg.zig");

test { // NOTE: here so build test run runs imported modules' tests.
    std.testing.refAllDecls(@This());
}
