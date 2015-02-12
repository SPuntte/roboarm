include <elbow_dimensions.scad>

use <elbow_servo_mount.scad>
use <servo.scad>
use <bearing.scad>

// Servo housing
translate([0, 0, -wall_thickness])
{
elbow_mount_base(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                 servo_ear_width, servo_arm_position, axle_position,
                 wall_thickness, wall_thickness);
}
elbow_mount_walls(servo_width, servo_height, servo_depth, servo_ear_position,
                servo_ear_thickness, servo_ear_width, servo_arm_position,
                wall_thickness, wall_extrude);