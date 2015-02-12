$fn = 50;
// All dimensions in millimeters (mm)

$epsilon = 0.05;
$padding = 0.2;
$padding2 = 2 * $padding;

servo_height = 20.4 + $padding2;
servo_width = 40.6 + $padding2;
servo_depth = 36.6 + $padding2;

servo_ear_width = 6.8 + $padding;
servo_ear_thickness = 2.7 + $padding2;
servo_ear_position = 26.9 + $padding;

servo_arm_position = 43 + $padding;
servo_arm_width = 21;

wall_thickness = 4;
wall_extrude = 2;
cord_size = [9.2, 3.8];

fork_thickness = wall_thickness;

// [outer radius, inner radius, height, outer ring thickness, inner ring thickness]
bearing_dims = [4.76+$padding2, 2.375-$padding, 3.15, 1.1, 1.1];

/*
[cone length, cone inner radius, cone outer radius
 axle length, axle inner radius, axle outer radius, nut diameter]
*/
axle_dims = [ 3, 8, 4,
             2*bearing_dims[2] + fork_thickness/3, 1.5+$padding, bearing_dims[1], 6.1];
axle_position = 10;

fork_axle_length = 5;
fork_axle_hole = 7;
fork_axle_aux_r = 7.85;
fork_axle_aux_hole = 1 + $padding2;

fork_width = 30;
fork_offset = 45;
fork_height = fork_axle_length + 2*fork_thickness + wall_thickness +
              servo_arm_position + axle_dims[0] + bearing_dims[2] - fork_thickness/3;