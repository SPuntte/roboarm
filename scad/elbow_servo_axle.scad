include <elbow_dimensions.scad>

use <elbow_servo_mount.scad>
use <servo.scad>
use <bearing.scad>

// Axle
rotate([0, -90, 0])
translate([separation/2, 0, 0])
elbow_mount_axle(servo_height, servo_ear_width, axle_position, axle_dims,
               wall_thickness, wall_extrude, cord_size);
