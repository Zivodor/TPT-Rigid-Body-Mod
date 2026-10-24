Changes from Box2D v3.1.1
=========================

This is Box2D v3.1.1 (commit 8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3), MIT licensed (see `LICENSE`), with these changes for Rigid Body Mod. Each is marked with a "Rigid Body Mod" comment in the source.

* `meson.build` is new: it builds Box2D as part of the mod.
* `src/constants.h`: `B2_SPECULATIVE_DISTANCE` and `B2_AABB_MARGIN` are 4 pixels (0.4 m at the mod's 10 pixels a metre) instead of 0.2 and 0.5 pixels. Bodies here move up to 10 pixels a step, so with the defaults a fast body could get most of the way into a small one before Box2D gave them a contact, and the small one was then pushed out sideways or not at all. Larger values (8 pixels and up) made perfectly elastic bounces lose height. Contacts with the terrain (walls and solids, built from tiles) still only count within Box2D's original 0.2 pixels: the mod filters them in a pre-solve callback (`TerrainPreSolve` in `src/simulation/RigidBodies.cpp`), since further ahead a body sliding along a floor bumps into the side of the next tile.
* `src/solver.c`: restitution is applied `B2_RESTITUTION_ITERATIONS` (10) times each step instead of once. Applied once, a body landing flat on two contact points, or on two shapes at once, bounces back faster than its restitution allows, since each point's correction disturbs the others. With a restitution of 1, a box bounced a third higher than it fell.
