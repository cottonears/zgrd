const std = @import("std");
const mem = std.mem;
const Vec2f = @Vector(2, f32);

/// A struct that can be used to compose images from basic shapes. Useful for testing.
pub const Canvas = struct {
    min: Vec2f,
    max: Vec2f,
    str: std.ArrayList(u8),
    const Self = @This();

    pub fn init(allocator: mem.Allocator, min: Vec2f, max: Vec2f, background_style: ShapeStyle) !Self {
        const str = try std.ArrayList(u8).initCapacity(allocator, 2000);
        var canvas: Self = .{ .min = min, .max = max, .str = str };
        errdefer canvas.str.deinit(allocator);
        try canvas.addRectangle(allocator, min, max, background_style);
        return canvas;
    }

    pub fn deinit(self: *Self, allocator: mem.Allocator) void {
        self.str.deinit(allocator);
    }

    pub fn addLine(self: *Self, allocator: mem.Allocator, start: Vec2f, end: Vec2f, style: ShapeStyle) !void {
        var sbuff: [256]u8 = undefined;
        const style_str = try style.getElementString(&sbuff);
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<line x1=\"{d:.3}\" y1=\"{d:.3}\" x2=\"{d:.3}\" y2=\"{d:.3}\" {s}/>",
            .{ start[0], start[1], end[0], end[1], style_str },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addPolyline(self: *Self, allocator: mem.Allocator, points: []Vec2f, style: ShapeStyle) !void {
        var sbuff: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuff);
        var pts_char_list = try std.ArrayList(u8).initCapacity(allocator, points.len * 12);
        defer pts_char_list.deinit(allocator);
        for (points) |p| {
            var buff: [32]u8 = undefined;
            const slice = buff[0..];
            const str = try std.fmt.bufPrint(slice, "{d:.3},{d:.3} ", .{ p[0], p[1] });
            try pts_char_list.appendSlice(allocator, str);
        }
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<polyline points=\"{s}\" {s}/>",
            .{ pts_char_list.items[0..], style_str },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addRectangle(self: *Self, allocator: mem.Allocator, start: Vec2f, end: Vec2f, style: ShapeStyle) !void {
        var sbuff: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuff);
        const d = end - start;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<rect x=\"{d:.3}\" y=\"{d:.3}\" width=\"{d:.3}\" height=\"{d:.3}\" {s}/>", //style=\"overflow: visible\"
            .{ start[0], start[1], d[0], d[1], style_str },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addCircle(self: *Self, allocator: mem.Allocator, centre: Vec2f, radius: f32, style: ShapeStyle) !void {
        var sbuff: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuff);
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<circle cx=\"{d:.3}\" cy=\"{d:.3}\" r=\"{d:.3}\" {s}/>",
            .{ centre[0], centre[1], radius, style_str },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addPolygon(self: *Self, allocator: mem.Allocator, points: []Vec2f, style: ShapeStyle) !void {
        var sbuff: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuff);
        var pts_char_list = try std.ArrayList(u8).initCapacity(allocator, points.len * 12);
        defer pts_char_list.deinit(allocator);
        for (points) |p| {
            var buff: [32]u8 = undefined;
            const slice = buff[0..];
            const str = try std.fmt.bufPrint(slice, "{d:.3},{d:.3} ", .{ p[0], p[1] });
            try pts_char_list.appendSlice(allocator, str);
        }
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<polygon points=\"{s}\" {s}/>",
            .{ pts_char_list.items[0..], style_str },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addText(self: *Self, allocator: mem.Allocator, centre: Vec2f, text: []const u8, font_size: f32, fill_hsl: [3]u9) !void {
        var sbuff: [256]u8 = undefined;
        const style_str = try std.fmt.bufPrint(
            &sbuff,
            "font-size=\"{d:.4}\" fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" text-anchor=\"middle\"",
            .{ font_size, fill_hsl[0], fill_hsl[1], fill_hsl[2] },
        );
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<text x=\"{d:.3}\" y=\"{d:.3}\" {s}>{s}</text>",
            .{ centre[0], centre[1], style_str, text },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    // caller owns the returned memory
    pub fn getSvg(self: Self, allocator: mem.Allocator) ![]u8 {
        const extent = self.max - self.min;
        const rendered_max_dim: f32 = 800.0;
        const safe_w = if (extent[0] != 0) extent[0] else 1.0;
        const safe_h = if (extent[1] != 0) extent[1] else 1.0;
        const aspect = safe_w / safe_h;
        const rendered_w = if (aspect >= 1.0) rendered_max_dim else rendered_max_dim * aspect;
        const rendered_h = if (aspect >= 1.0) rendered_max_dim / aspect else rendered_max_dim;
        const svg_start = try std.fmt.allocPrint(
            allocator,
            "<svg viewBox=\"{d:.3} {d:.3} {d:.3} {d:.3}\" width=\"{d:.3}\" height=\"{d:.3}\" xmlns=\"http://www.w3.org/2000/svg\">",
            .{ self.min[0], self.min[1], extent[0], extent[1], rendered_w, rendered_h },
        );
        defer allocator.free(svg_start);

        var text = try std.ArrayList(u8).initCapacity(allocator, svg_start.len + self.str.items.len + 8);
        try text.appendSlice(allocator, svg_start);
        try text.appendSlice(allocator, self.str.items);
        try text.appendSlice(allocator, "\n</svg>");

        return text.toOwnedSlice(allocator);
    }

    pub fn writeHtml(
        self: *Self,
        allocator: mem.Allocator,
        io: std.Io,
        filename: []const u8,
        clear: bool,
    ) !void {
        const html_start = "<!DOCTYPE html>\n<html><body>";
        const html_end = "</body></html>";
        const svg_body = try self.getSvg(allocator);
        defer allocator.free(svg_body);

        if (std.fs.path.dirname(filename)) |dir| {
            try std.Io.Dir.cwd().createDirPath(io, dir);
        }
        var file = try std.Io.Dir.cwd().createFile(io, filename, .{});
        defer file.close(io);
        var buffer: [4096]u8 = undefined;
        var buf_writer = file.writer(io, &buffer);
        var writer = &buf_writer.interface;
        try writer.print("{s}\n{s}\n{s}", .{ html_start, svg_body, html_end });
        try buf_writer.flush();
        if (clear) try self.str.resize(allocator, 0);
    }
};

pub const ShapeStyle = struct {
    fill_active: bool = false,
    fill_hsl: [3]u9 = .{ 0, 0, 0 },
    fill_opacity: f32 = 1.0,
    stroke_active: bool = true,
    stroke_hsl: [3]u9 = .{ 0, 0, 0 },
    stroke_width: f32 = 1,
    stroke_opacity: f32 = 1.0,
    stroke_dashed: bool = false,

    pub fn getElementString(style: ShapeStyle, buff_ptr: []u8) ![]u8 {
        var len: usize = 0;
        if (style.fill_active) {
            len = (try std.fmt.bufPrint(
                buff_ptr,
                "fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" ",
                .{ style.fill_hsl[0], style.fill_hsl[1], style.fill_hsl[2] },
            )).len;
        } else {
            len = (try std.fmt.bufPrint(
                buff_ptr,
                "fill=\"none\" ",
                .{},
            )).len;
        }
        if (style.stroke_active) {
            len += (try std.fmt.bufPrint(
                buff_ptr[len..],
                "stroke=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" stroke-width=\"{d:.4}\" ",
                .{ style.stroke_hsl[0], style.stroke_hsl[1], style.stroke_hsl[2], style.stroke_width },
            )).len;

            if (style.stroke_opacity < 1.0) {
                len += (try std.fmt.bufPrint(
                    buff_ptr[len..],
                    "stroke-opacity=\"{d:.4}\" ",
                    .{style.stroke_opacity},
                )).len;
            }
            if (style.stroke_dashed) {
                len += (try std.fmt.bufPrint(
                    buff_ptr[len..],
                    "stroke-dasharray=\"{},{}\" ",
                    .{ 2 * style.stroke_width, 2 * style.stroke_width },
                )).len;
            }
        }
        return buff_ptr[0..len];
    }
};

const DEFAULT_HUE_RANGE = [_]u9{ 0, 360 };
const DEFAULT_SAT_RANGE = [_]u9{ 50, 70 };
const DEFAULT_LT_RANGE = [_]u9{ 40, 60 };

pub const RandomHslPalette = struct {
    prng: std.Random.DefaultPrng,
    hsl_colours: [][3]u9,
    h_min: u9 = DEFAULT_HUE_RANGE[0],
    h_max: u9 = DEFAULT_HUE_RANGE[1],
    s_min: u9 = DEFAULT_SAT_RANGE[0],
    s_max: u9 = DEFAULT_SAT_RANGE[1],
    l_min: u9 = DEFAULT_LT_RANGE[0],
    l_max: u9 = DEFAULT_LT_RANGE[1],

    const Self = @This();

    pub fn init(allocator: mem.Allocator, num_colours: u8, seed: usize) !Self {
        var pal = Self{
            .prng = std.Random.DefaultPrng.init(seed),
            .hsl_colours = try allocator.alloc([3]u9, num_colours),
        };
        pal.regenerate();
        return pal;
    }

    pub fn deinit(self: *Self, allocator: mem.Allocator) void {
        allocator.free(self.hsl_colours);
    }

    pub fn getRandomColour(self: *Self) [3]u9 {
        const random = self.prng.random();
        return .{
            random.intRangeLessThan(u9, self.h_min, self.h_max),
            random.intRangeLessThan(u9, self.s_min, self.s_max),
            random.intRangeLessThan(u9, self.l_min, self.l_max),
        };
    }

    /// The colours will be evenly spaced within the hue range, but have the same lightness + saturation.
    pub fn regenerate(self: *Self) void {
        const hue_range = self.h_max - self.h_min;
        const hue_inc: u9 = @truncate(hue_range / self.hsl_colours.len);
        var col = self.getRandomColour();
        for (0..self.hsl_colours.len) |i| {
            self.hsl_colours[i] = col;
            col[0] = @intCast((@as(u16, col[0]) + hue_inc) % self.h_max);
        }
    }
};

// testing code
const testing = std.testing;
const canvas_min: Vec2f = .{ 0, 0 };
const canvas_max: Vec2f = .{ 800, 600 };
const bg_style = ShapeStyle{
    .fill_active = true,
    .fill_hsl = .{ 0, 0, 90 },
    .stroke_hsl = .{ 0, 0, 0 },
};

test "random colours" {
    var pal = try RandomHslPalette.init(std.testing.allocator, 4, 0);
    defer pal.deinit(std.testing.allocator);
    for (0..4) |i| {
        const current_hsl = pal.hsl_colours[i];
        const next_hsl = pal.hsl_colours[(i + 1) % 4];
        try testing.expect(current_hsl[0] != next_hsl[0]); // hue shifted
        try testing.expect(current_hsl[1] == next_hsl[1]); // unchanged
        try testing.expect(current_hsl[2] == next_hsl[2]); // unchanged
    }
}

test "add elements" {
    var points = [_]Vec2f{
        [_]f32{ 0, 100 },
        [_]f32{ 200, 300 },
        [_]f32{ 100, 150 },
    };
    var canvas = try Canvas.init(testing.allocator, canvas_min, canvas_max, bg_style);
    defer canvas.deinit(testing.allocator);
    var pal = try RandomHslPalette.init(std.testing.allocator, 4, 0);
    defer pal.deinit(std.testing.allocator);

    const style_0 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[0], .stroke_dashed = true };
    const style_1 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[1] };
    const style_2 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[2] };
    const style_3 = ShapeStyle{ .stroke_hsl = pal.hsl_colours[3] };
    try canvas.addRectangle(testing.allocator, [_]f32{ -200, 0 }, [_]f32{ -200, 200 }, style_0);
    try canvas.addPolygon(testing.allocator, points[0..], style_1);
    try canvas.addCircle(testing.allocator, [_]f32{ 0, 200 }, 10.0, style_2);
    try canvas.addLine(testing.allocator, [_]f32{ -200, -100 }, [_]f32{ 200, 300 }, style_3);
}
