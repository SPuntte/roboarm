include <elbow_dimensions.scad>

use <elbow_servo_mount.scad>
use <servo.scad>
use <bearing.scad>

separation = 50;

union()
{
  // Servo fork
  translate([0, 100, 0])
  {
    elbow_servo_fork(fork_width, fork_height, fork_offset, servo_arm_width, fork_axle_length, fork_axle_hole, fork_axle_aux_r, fork_axle_aux_hole, fork_thickness, axle_dims, bearing_dims);
   
    // Bearings
    translate([fork_height - fork_axle_length - fork_thickness - 2*bearing_dims[2], 0, 0])
    rotate([0, 90, 0])
    {
      translate([0, 0, -fork_thickness])
      union()
      {
        bearing(bearing_dims);
        translate([0, 0, fork_thickness + 3*bearing_dims[2]])
        {
          bearing(bearing_dims);
        }
      }
    }
  }

  // Servo
  translate([0, 0, 2*separation/3])
  union()
  {
    translate([-servo_arm_position, axle_position, servo_height/2])
    {
      rotate([0, -90, 0])
      {
        servo(servo_arm_position, servo_width, servo_depth,
              servo_height, servo_width + 2*servo_ear_width,
              servo_ear_position, axle_position);
      }
    }

    translate([-servo_arm_position, axle_position, servo_height/2])
    {
      rotate([0, 90, 0])
      {
        cylinder(h=2, d=servo_arm_width);
      }
    }
  }

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

  // Axle
  translate([separation/2, 0, 0])
  elbow_mount_axle(servo_height, servo_ear_width, axle_position, axle_dims,
                   wall_thickness, wall_extrude, cord_size);

  // Housing top
  translate([0, 0, servo_height + wall_extrude + separation])
  {
    elbow_mount_lid(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                    servo_ear_width, servo_arm_position, axle_position,
                    wall_thickness, wall_extrude, cord_size);
  }

  //============================================================================
  // All together
  translate([150, 0, 0])
  union()
  {
    // Bearings
    translate([wall_thickness + axle_dims[0], 0, 0])
    {
      rotate([0, 90, 0])
      {
        union()
        {
          bearing(bearing_dims);
          translate([0, 0, bearing_dims[2] + fork_thickness/3])
          {
            bearing(bearing_dims);
          }
        }
      }
    }

    // Fork
    translate([-servo_arm_position, 0, 0])
    rotate([-90, 0, 0])
    elbow_servo_fork(fork_width, fork_height, fork_offset, servo_arm_width, fork_axle_length, fork_axle_hole, fork_axle_aux_r, fork_axle_aux_hole, fork_thickness, axle_dims, bearing_dims);

    rotate([80, 0, 0])
    translate([0, -axle_position, -servo_height/2])
    union()
    {
      // Servo
      translate([-servo_arm_position, axle_position, servo_height/2])
      {
        rotate([0, -90, 0])
        {
          servo(servo_arm_position, servo_width, servo_depth,
                servo_height, servo_width + 2*servo_ear_width,
                servo_ear_position, axle_position);
        }
      }

      // Servo arm
      translate([-servo_arm_position, axle_position, servo_height/2])
      {
        rotate([0, 90, 0])
        {
          cylinder(h=2, d=servo_arm_width);
        }
      }

      // Elbow servo housing
      translate([0, 0, -wall_thickness])
      {
        elbow_mount_base(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                         servo_ear_width, servo_arm_position, axle_position,
                         wall_thickness, wall_thickness);
      }
      elbow_mount_walls(servo_width, servo_height, servo_depth, servo_ear_position,
                        servo_ear_thickness, servo_ear_width, servo_arm_position,
                        wall_thickness, wall_extrude);

      elbow_mount_axle(servo_height, servo_ear_width, axle_position, axle_dims,
                       wall_thickness, wall_extrude, cord_size);

      translate([0, 0, servo_height + wall_extrude])
      {
        elbow_mount_lid(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                        servo_ear_width, servo_arm_position, axle_position,
                        wall_thickness, wall_extrude, cord_size);
      }
    }
  }
}