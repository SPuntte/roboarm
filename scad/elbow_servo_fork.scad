include <elbow_dimensions.scad>

use <elbow_servo_mount.scad>
use <servo.scad>
use <bearing.scad>

union()
{
	rotate([180, 0, 0])
	elbow_servo_fork(fork_width, fork_height, fork_offset, servo_arm_width, fork_axle_length, fork_axle_hole, fork_axle_aux_r, fork_axle_aux_hole, fork_thickness, axle_dims, bearing_dims);

	translate([$epsilon/2 - (fork_axle_length + fork_thickness) / 3, 0, 0])
	cube([$epsilon, fork_axle_hole/2, fork_axle_hole], center=true);

	translate([$epsilon/2 -2*(fork_axle_length + fork_thickness) / 3, 0, 0])
	cube([$epsilon, fork_axle_hole/2, fork_axle_hole], center=true);

	translate([$epsilon -(fork_axle_length + fork_thickness), 0, 0])
	cube([$epsilon, fork_axle_hole/2, fork_axle_hole], center=true);

	translate([-$epsilon, 0, 0])
	cube([$epsilon, fork_axle_hole/2, fork_axle_hole], center=true);

	translate([fork_height + 2*bearing_dims[2]/3 -(fork_axle_length + fork_thickness) - 0.4, 0, 0])
	cube([$epsilon, bearing_dims[0], 2*bearing_dims[0]], center=true);

	translate([fork_height - bearing_dims[2] -(fork_axle_length + fork_thickness) - 2.55, 0, 0])
	cube([$epsilon, bearing_dims[0], 2*bearing_dims[0]], center=true);

	translate([fork_height - 2 -(fork_axle_length + fork_thickness), 0, 0])
	cube([$epsilon, 1.75*bearing_dims[1], 4*bearing_dims[1]], center=true);
}