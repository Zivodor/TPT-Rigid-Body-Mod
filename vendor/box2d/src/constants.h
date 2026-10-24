// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

extern float b2_lengthUnitsPerMeter;

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
#define B2_HUGE ( 100000.0f * b2_lengthUnitsPerMeter )

// Maximum parallel workers. Used to size some static arrays.
#define B2_MAX_WORKERS 64

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
#define B2_GRAPH_COLOR_COUNT 12

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// Normally this is 0.5cm.
// @warning modifying this can have a significant impact on stability
#define B2_LINEAR_SLOP ( 0.005f * b2_lengthUnitsPerMeter )

// Maximum number of simultaneous worlds that can be allocated
#ifndef B2_MAX_WORLDS
#define B2_MAX_WORLDS 128
#endif

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// @warning increasing this to 0.5f * b2_pi or greater will break continuous collision.
#define B2_MAX_ROTATION ( 0.25f * B2_PI )

// Box2D uses limited speculative collision. This reduces jitter.
// Normally this is 2cm.
// @warning modifying this can have a significant impact on performance and stability
// Rigid Body Mod: 4 pixels (0.4 m at 10 pixels a metre) instead of 0.2. Bodies
// here move up to 10 pixels a step, so with the default a fast one could get
// most of the way into a small one before they had a contact at all; then
// the small one was pushed out sideways, or not at all.
#define B2_SPECULATIVE_DISTANCE ( 0.4f * b2_lengthUnitsPerMeter )

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
// Normally this is 5cm.
// @warning modifying this can have a significant impact on performance
// Rigid Body Mod: as big as the speculative distance, so that bodies that
// close are paired up to have a contact.
#define B2_AABB_MARGIN ( 0.4f * b2_lengthUnitsPerMeter )

// The time that a body must be still before it will go to sleep. In seconds.
#define B2_TIME_TO_SLEEP 0.5f

// The default joint constraint hertz
#define B2_JOINT_CONSTRAINT_HERTZ 60.0f

// The default joint constraint damping ratio
#define B2_JOINT_CONSTRAINT_DAMPING_RATIO 2.0f

enum b2TreeNodeFlags
{
	b2_allocatedNode = 0x0001,
	b2_enlargedNode = 0x0002,
	b2_leafNode = 0x0004,
};
