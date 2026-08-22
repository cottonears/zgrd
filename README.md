# zgrd

Zgrd (pronounced 'zigrid') is a library for 2D spatial data structures that aims to do be three things:
- Simple
- Lightweight
- Efficient

Currently zgrd offers just one data structure, `SquareTree`, for fast queries in 2D scenes.
It is tailored to realtime applications where thousands of bodies are fairly uniformly distributed (e.g., RTS- and MMORPG-style games, life / particle simulators).
It aims to extend the commonly-used (for good reason!) uniform grid approach in a useful way.
A square tree won't be suitable for every application, it is likely to be much slower than alternatives for very sparse scenes.
More data structures are planned in future, see the [Roadmap](#roadmap).

## Prerequisites
Zig 0.16.

## Installation

Use `zig fetch` to import the zgrd package to your project:
``` sh
zig fetch --save git+https://github.com/cottonears/zgrd
```
Then register it as a dependency of your app in `build.zig.zon`:
``` zig
const zgrd_dep = b.dependency("zgrd", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("zgrd", zgrd_dep.module("zgrd"));
```

## Simple working example
``` zig
const std = @import("std");
const zgrd = @import("zgrd");
const Ball2f = zgrd.volume.Ball2f;
const Box2f = zgrd.volume.Box2f;
const Indexer = zgrd.index.Indexer2f(4, 2, 1, .Zigzag);
const SquareTree = zgrd.square_tree.SquareTree(Indexer, Box2f, u16);

pub fn main(init: std.process.Init) !void {
    const arena: std.mem.Allocator = init.arena.allocator();
    const min = @Vector(2, f32){ 0, 0 };
    const max = @Vector(2, f32){ 16, 10 };
    var tree = try SquareTree.init(arena, min, max, 50_000);
    defer tree.deinit(arena);

    var entity_aabbs: [3]Box2f = .{
        .{ .min = .{ 1.0, 1.0 }, .max = .{ 2.0, 3.0 } },
        .{ .min = .{ 1.5, 0.0 }, .max = .{ 1.8, 4.0 } },
        .{ .min = .{ 1.2, 2.0 }, .max = .{ 1.3, 2.5 } },
    };
    var entity_ids: [3]u16 = .{ 0, 1, 2 };

    // square trees can be rebuilt cheaply: clear and update every frame
    tree.clearStoredVolumes();
    try tree.addVolumes(entity_aabbs[0..], entity_ids[0..]);
    tree.updateBounds();

    // check for overlaps with an external volume with findOverlaps
    var query_buff: [3]u16 = undefined; // NOTE: slice of u16s
    const query_ball = Ball2f{ .centre = .{ 4, 4 }, .radius = 3 };
    const query_ids = try tree.findOverlaps(&query_buff, query_ball);
    for (query_ids) |id| {
        std.debug.print("Query ball overlaps with {d}.\n", .{id});
    }

    // check for overlaps betweeen stored objects with findSelfOverlaps
    var overlaps_buff: [6][2]u16 = undefined; // NOTE: slice of u16 pairs
    const entity_pairs = try tree.findSelfOverlaps(&overlaps_buff);
    for (entity_pairs) |p| {
        std.debug.print("Entity overlap between {d} and {d}.\n", .{ p[0], p[1] });
    }
}
```

There is a companion project [`zgrd-demo`](https://github.com/cottonears/zgrd-demo) that shows a more complete example of using zgrd within a game / simulation loop.


## Volumes
Several types of volumes supported, describe them and contrast storable vs non-storable.

## SquareTree
A square tree is a uniform grid where each top-level (level 0) cell has a a bounding volume hierachy (BVH) tree beneath it.
A consis
Adding the BVH allows for more flexible queries, and better performance if some cells become densely packed.
The bottom (leaf) level grid is formed by dividing a square region of the 2D plane into smaller cells of equal size.
When volumes are added to a square tree, they are 'binned' into one of these leaf cells based on their centre position.
After all relevant volumes have been binned into their leaf cells, an axis-aligned bounding box (AABB) is fit around the volumes stored in each cell.
Then, a second level of AABBs is fit around a number of neighbouring cells' bounding boxes.
This is repeated iteratively until the top layer of AABBs (at level 0) has been created.
For efficiency, the hierachy is built using a recursive indexing technique; see [Indexing](#indexing) for more details.

The square tree data structure uses a single slice to store all volumes (from all leaf cells) in a one large block of memory.
This has the following benefits:
- The square tree will allocate heap memory exactly once (on init). Adding volumes to the tree after initialisation will never trigger a heap allocation, but it may result in a `CapacityExceeded` error (if the initial capacity has been exhausted).
- The share of the overall tree's capacity used by each leaf cell is flexible and will adapt as required to the scene. This results in lower memory usage (+ safer runtime behaviour) than a naiive approach where each cell has a separate backing array/slice.
- Volumes within the same leaf are stored in contiguous memory; this is important for performance.
Neighbouring leaves' volumes are also frequently adjacent in memory, though it's not clear if this impacts performance in the current implementation (TODO: investigate!).

(A paragraph about querying the tree and BFS/DTT here)

## Indexing
(Write about the recursive indexing techniques used)


## Sizing your square tree
(Tips + directions to how to size trees and use the benchmark tool on data representative of use-case, or (even better) directly imported data from a real scene).

## Roadmap
In no particular order:

- Implement helper functions for creating conservative bounding volumes for moving bodies. Something along the lines of `getExpandedVolume(V, vol, velocity, time_step)`.
- Add layered_tree that wraps several trees (e.g., static + dynamic, player1 + player2) and allows for ergonomic in-tree and cross-tree queries.
- Add multi-threading support.
- Experiment with a dynamic-depth 2D linear BVH along the lines of:
https://gamma-web.iacs.umd.edu/papers/documents/articles/2009/lauterbach09.pdf
https://www.pbr-book.org/4ed/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies
