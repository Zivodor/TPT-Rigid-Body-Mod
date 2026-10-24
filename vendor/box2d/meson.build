# Box2D v3.1.1 (https://github.com/erincatto/box2d, commit 8c661469c9507d3ad6fbd2fea3f1aa71669c2fe3),
# vendored unmodified apart from leaving out its samples, tests, docs and CMake files. MIT licensed, see LICENSE.
add_languages('c', native: false, required: true)

box2d_c_args = []
# Box2D's own build uses CMake's default of C17 with extensions, which it needs for clock_gettime and friends.
box2d_c_std = meson.get_compiler('c').get_id() in [ 'msvc', 'clang-cl' ] ? 'c17' : 'gnu17'
if host_platform == 'emscripten' or not (is_x86 or host_arch == 'aarch64')
	box2d_c_args += [ '-DBOX2D_DISABLE_SIMD' ]
endif

box2d_dep = declare_dependency(
	include_directories: include_directories('include', is_system: true),
	link_with: static_library(
		'box2d',
		sources: files(
			'src/aabb.c',
			'src/arena_allocator.c',
			'src/array.c',
			'src/bitset.c',
			'src/body.c',
			'src/broad_phase.c',
			'src/constraint_graph.c',
			'src/contact.c',
			'src/contact_solver.c',
			'src/core.c',
			'src/distance.c',
			'src/distance_joint.c',
			'src/dynamic_tree.c',
			'src/geometry.c',
			'src/hull.c',
			'src/id_pool.c',
			'src/island.c',
			'src/joint.c',
			'src/manifold.c',
			'src/math_functions.c',
			'src/motor_joint.c',
			'src/mouse_joint.c',
			'src/mover.c',
			'src/prismatic_joint.c',
			'src/revolute_joint.c',
			'src/sensor.c',
			'src/shape.c',
			'src/solver.c',
			'src/solver_set.c',
			'src/table.c',
			'src/timer.c',
			'src/types.c',
			'src/weld_joint.c',
			'src/wheel_joint.c',
			'src/world.c',
		),
		include_directories: include_directories('include', 'src'),
		c_args: box2d_c_args,
		override_options: [ 'c_std=' + box2d_c_std, 'warning_level=0' ],
		pic: host_platform == 'android',
	),
)
