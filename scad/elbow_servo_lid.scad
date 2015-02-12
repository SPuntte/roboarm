include <elbow_dimensions.scad>

use <elbow_servo_mount.scad>
use <servo.scad>
use <bearing.scad>

// Housing top
rotate([180, 0, 0])
translate([0, 0, servo_height + wall_extrude + separation])
{
elbow_mount_lid(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                servo_ear_width, servo_arm_position, axle_position,
                wall_thickness, wall_extrude, cord_size);
}