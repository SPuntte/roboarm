use <triangle_truss.scad>

module elbow_mount_base(servo_width, servo_depth, servo_ear_position, servo_ear_thickness, servo_ear_width, servo_arm_position, axle_position, wall_thickness, thickness)
{
  bevel = servo_depth - (servo_ear_position + servo_ear_thickness + wall_thickness);
  linear_extrude(height = thickness)
  {
    polygon([[wall_thickness, -(servo_ear_width + wall_thickness)],
             [wall_thickness, 2*axle_position ],
             [-servo_ear_position + wall_thickness, servo_width + servo_ear_width + wall_thickness],
             [-servo_depth + bevel, servo_width + servo_ear_width + wall_thickness],
             [-servo_depth, servo_width + wall_thickness],
             [-servo_depth, 0], [-servo_arm_position, -servo_ear_width],
             [-servo_arm_position, -(servo_ear_width + wall_thickness)]]);
  }
}

module servo_ear_holder(servo_height, servo_ear_thickness, servo_ear_width, wall_thickness, wall_extrude)
{
  difference()
  {
    cube(size=[2*wall_thickness + servo_ear_thickness,
               servo_ear_width + wall_thickness,
               servo_height + wall_extrude],
         center=false);
    translate([wall_thickness, wall_thickness, -$epsilon])
    cube(size=[servo_ear_thickness, servo_ear_width + $epsilon,
               servo_height + wall_extrude + 2*$epsilon],
         center=false);
  }
}

module elbow_mount_walls(servo_width, servo_height, servo_depth, servo_ear_position, servo_ear_thickness, servo_ear_width, servo_arm_position, wall_thickness, wall_extrude)
{
  bevel = servo_depth - (servo_ear_position + servo_ear_thickness + wall_thickness);

  beam_width = 4;
  edge_extrude = 2;
  num_inner_beams = 6;
  inner_beam_angle = 45;
  inner_width = 25;
  screw_d = 3;
  screw_position = 1;

  union()
  {
    translate([-servo_depth + bevel, -(wall_thickness + servo_ear_width), 0])
    servo_ear_holder(servo_height, servo_ear_thickness, servo_ear_width,
                     wall_thickness, wall_extrude);

    translate([-servo_depth + bevel, servo_width + wall_thickness + servo_ear_width])
    mirror([0, 1, 0])
    {
      servo_ear_holder(servo_height, servo_ear_thickness, servo_ear_width,
                       wall_thickness, wall_extrude);
    }

    translate([-servo_ear_position + wall_thickness - $epsilon, -wall_extrude])
    {
      cube(size=[servo_ear_position - wall_thickness - $padding2 + $epsilon, wall_extrude, wall_extrude], center=false);
    }

    /*
    translate([-servo_arm_position, -(wall_thickness + servo_ear_width)])
    {
      cube(size=[servo_arm_position + wall_thickness, wall_thickness, servo_height + wall_extrude], center=false);
    }
    */

    intersection()
    {
      translate([-servo_arm_position + 10, -(wall_thickness + servo_ear_width)])
      {
        cube(size=[servo_arm_position + wall_thickness, wall_thickness, servo_height + wall_extrude], center=false);
      }
      translate([10, -servo_ear_width, servo_height + wall_extrude])
      {
        rotate([90, 0, 0])
        {
          rotate([0, 0, 180])
          {
            triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position);  
          }
        }
      }
    }

    translate([-servo_arm_position, -(wall_thickness + servo_ear_width)])
    {
      union()
      {
        cube([servo_arm_position + wall_thickness, wall_thickness, 4.2]);
        cube([22, wall_thickness, servo_height + wall_extrude], center=false);
        translate([wall_thickness + servo_arm_position-5.5, 0, 0])
        {
          cube([5.5, wall_thickness, servo_height + wall_extrude]);
        }
      }
    }    

    
    /*
    translate([10, -(servo_ear_width + wall_thickness + 1), servo_height + wall_extrude])
    {
      rotate([90, 0, 0])
      {
        rotate([0, 0, 180])
        {
          triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position);  
        }
      }
    }
    */
  }
}

module elbow_mount_axle(servo_height, servo_ear_width, axle_position, axle_dims, wall_thickness, wall_extrude, cord_size)
{
  servo_height = servo_height - $padding2;
  servo_ear_width = servo_ear_width - $padding;

  difference()
  {
    union()
    {
      translate([0, -servo_ear_width, 0])
      {
        cube(size=[wall_thickness, 2*axle_position + servo_ear_width, servo_height + wall_extrude], center=false);
      }
      translate([wall_thickness - $epsilon, axle_position, servo_height/2])
      {
        rotate([0, 90, 0])
        {
          cylinder(h=axle_dims[0] + $epsilon, r1=axle_dims[1], r2=axle_dims[2]);
        }
      }
      translate([wall_thickness + axle_dims[0] - $epsilon, axle_position, servo_height/2])
      rotate([0, 90, 0])
      {
        cylinder(h=axle_dims[3] + $epsilon, r=axle_dims[5]);
      }
    }

    translate([0, axle_position, servo_height/2])
    rotate([0, 90, 0])
    {
      translate([0, 0, -$epsilon])
      {
        union()
        {
          cylinder(h=wall_thickness + axle_dims[0] + axle_dims[3] + 2*$epsilon, r=axle_dims[4]);
          intersection()
          {
            cylinder(h=2.25*axle_dims[4] + $padding + $epsilon, r=axle_dims[6]/2 + $padding + $padding2, $fn=6);
            cylinder(h=2.25*axle_dims[4] + $padding + $epsilon, r1=1.1*axle_dims[6], r2 = axle_dims[4], $fn=6);
          }
        }
      }
    }
  }
}

module elbow_mount_lid(servo_width, servo_depth, servo_ear_position, servo_ear_thickness, servo_ear_width, servo_arm_position, axle_position, wall_thickness, wall_extrude, cord_size)
{
  bevel = servo_depth - (servo_ear_position + servo_ear_thickness + wall_thickness);
  difference()
  {
    union()
    {
      elbow_mount_base(servo_width, servo_depth, servo_ear_position, servo_ear_thickness,
                          servo_ear_width, servo_arm_position, axle_position,
                          wall_thickness, wall_extrude);
      
      translate([0, 0, -wall_extrude])
      {
        linear_extrude(height = wall_extrude + $epsilon)
        {
          polygon([[-$padding2, $padding2 - servo_ear_width], [-$padding2, axle_position],
                   [-servo_ear_position + wall_thickness, servo_width - $padding2],
                   [-servo_ear_position - $padding2, servo_width - $padding2],
                   [-servo_ear_position - $padding2, servo_width + servo_ear_width - $padding2],
                   [-servo_ear_position - servo_ear_thickness + $padding2, servo_width + servo_ear_width - $padding2],
                   [-servo_ear_position - servo_ear_thickness + $padding2, servo_width - $padding2],
                   [-servo_depth, servo_width - $padding2], [-servo_depth, 0],
                   [-servo_arm_position + $padding2, -servo_ear_width + $padding2],
                   [-servo_ear_position - servo_ear_thickness - wall_thickness - $padding2, -servo_ear_width + $padding2],
                   [-servo_ear_position - servo_ear_thickness - wall_thickness - $padding2, $padding2],
                   [-servo_ear_position - servo_ear_thickness + $padding2, $padding2],
                   [-servo_ear_position - servo_ear_thickness + $padding2, $padding2 - servo_ear_width],
                   [-servo_ear_position - $padding2, $padding2 - servo_ear_width],
                   [-servo_ear_position - $padding2, $padding2],
                   [-servo_ear_position + wall_thickness + $padding2, $padding2],
                   [-servo_ear_position + wall_thickness + $padding2, $padding2 - servo_ear_width]
                   ]);
        }
      }
      translate([-servo_ear_position + wall_thickness + $padding2, -wall_extrude, -2*wall_extrude])
      {
        cube(size=[servo_ear_position - wall_thickness - 2*$padding2,
                   wall_extrude, wall_extrude + $epsilon],
             center=false);
      }
    }
    translate([-cord_size[0] - $padding2 + $epsilon, $padding2 - servo_ear_width - $epsilon, -(2*wall_extrude + $epsilon)])
    cube(size=[cord_size[0], cord_size[1], 3*wall_extrude + 2*$epsilon], center=false);
  }
}

module elbow_servo_fork(fork_width, fork_height, fork_offset, servo_arm_width, fork_axle_length, fork_axle_hole, fork_axle_aux_r, fork_axle_aux_hole, fork_thickness, axle_dims, bearing_dims)
{
  strafe_angle = 80;
  strafe_length = 30;
  strafe_x = strafe_length * cos(strafe_angle);
  strafe_y = strafe_length * sin(strafe_angle);

  beam_width = 5;
  edge_extrude = 5;
  num_inner_beams = 4;
  inner_beam_angle = 45;
  inner_width = 30;
  screw_d = 3;
  screw_position = 1;

  strut_r1 = 7;
  strut_r2 = 12;
  strut_h2 = 15;

  union()
  {
    difference()
    {
      union()
      {
        rotate([0, -90, 0])
        {
          cylinder(h=fork_axle_length, r1=servo_arm_width/2, r2=fork_width/2);
        }
        translate([-fork_axle_length, 0, 0])
        {
          union()
          {
            rotate([strafe_angle, 0, 0])
            translate([-fork_thickness, 0, -fork_width/2])
            cube(size=[fork_thickness, strafe_length, fork_width], center=false);
            rotate([0, -90, 0])
            {
              cylinder(h=fork_thickness, r=fork_width/2);
            }
          }
        }
      }
      union()
      {
        rotate([0, -90, 0])
        {
          translate([0, 0, -$epsilon])
          cylinder(h=fork_axle_length + fork_width + 2*$epsilon, r=fork_axle_hole/2);
        }
        translate([0, fork_axle_aux_r, 0])
        {
          rotate([0, -90, 0])
          {
            translate([0, 0, -$epsilon])
            {
              cylinder(h=fork_axle_length + fork_width + 2*$epsilon, r=fork_axle_aux_hole/2);
            }
          }
        }
        translate([0, -fork_axle_aux_r, 0])
        {
          rotate([0, -90, 0])
          {
            translate([0, 0, -$epsilon])
            {
              cylinder(h=fork_axle_length + fork_width + 2*$epsilon, r=fork_axle_aux_hole/2);
            }
          }
        }
        translate([0, 0, fork_axle_aux_r])
        {
          rotate([0, -90, 0])
          {
            translate([0, 0, -$epsilon])
            {
              cylinder(h=fork_axle_length + fork_width + 2*$epsilon, r=fork_axle_aux_hole/2);
            }
          }
        }
        translate([0, 0, -fork_axle_aux_r])
        {
          rotate([0, -90, 0])
          {
            translate([0, 0, -$epsilon])
            {
              cylinder(h=fork_axle_length + fork_width + 2*$epsilon, r=fork_axle_aux_hole/2);
            }
          }
        }
      }
    }

    translate([-fork_axle_length, strafe_x, strafe_y])
    {
      union()
      {
        rotate([0, -90, 0])
        {
          cylinder(h=fork_thickness, r=fork_width/2);
        }
        translate([-fork_thickness, 0, -fork_width/2])
        {
          cube(size=[fork_thickness, fork_offset - strafe_x, fork_width], center=false);
        }
      }
    }

    translate([-(fork_axle_length + fork_thickness), fork_offset, strafe_y-fork_width/2])
    {
      union()
      {
        //cube([fork_height, fork_thickness, fork_width]);
        cube([fork_height, fork_thickness, 5]);
        cube([7, fork_thickness, fork_width]);
        translate([34, 0, 0])
        cube([fork_height - 34, fork_thickness, fork_width]);

        /*
        translate([-8, fork_thickness + 1, fork_width])
        rotate([-90, 0, 0])
        triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position);
        */

        intersection()
        {
          translate([-10, 0, -fork_width])
          cube([fork_height, fork_thickness, fork_width*2]);
          translate([-8, 0, fork_width])
          {
            rotate([-90, 0, 0])
            {
              triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position);
            }
          }
        }

        translate([fork_thickness - $epsilon, $epsilon, 0])
        {
          
          intersection()
          {
            cylinder(h=strut_r1, r1=strut_r1, r2=0, center=false, $fn=4);

            linear_extrude(height = 8)
            {
              polygon([[0, 0], [strut_r1, 0], [0, -strut_r1]]);
            }
          }
          
          translate([0, 0, fork_width])
          mirror([0, 0, 1])
          intersection()
          {
            cylinder(h=strut_h2, r1=strut_r2, r2=0, center=false, $fn=4);

            linear_extrude(height = strut_h2)
            {
              polygon([[0, 0], [strut_r2, 0], [0, -strut_r2]]);
            }
          }

          translate([fork_height - 2*fork_thickness + 2*$epsilon, 0, 0])
          mirror([1, 0, 0])
          union()
          {
            intersection()
            {
              cylinder(h=strut_h2, r1=strut_r2, r2=0, center=false, $fn=4);

              linear_extrude(height = strut_h2)
              {
                polygon([[0, 0], [strut_r2, 0], [0, -strut_r2]]);
              }
            }

            translate([0, 0, fork_width])
            mirror([0, 0, 1])
            intersection()
            {
              cylinder(h=strut_h2, r1=strut_r2, r2=0, center=false, $fn=4);

              linear_extrude(height = strut_h2)
              {
                polygon([[0, 0], [strut_r2, 0], [0, -strut_r2]]);
              }
            }
          }
        }
      }
    }




    translate([fork_height - fork_thickness - fork_axle_length, strafe_x, strafe_y])
    {
      union()
      {
        rotate([0, -90, 0])
        {
          cylinder(h=fork_thickness, r=fork_width/2);
        }
        translate([-fork_thickness, 0, -fork_width/2])
        {
          cube(size=[fork_thickness, fork_offset - strafe_x, fork_width], center=false);
        }
      }
    }

    translate([fork_height - 2*fork_thickness - fork_axle_length, 0, 0])
    {
      difference()
      {
        union()
        {
          
          rotate([strafe_angle, 0, 0])
          translate([0, 0, -fork_width/2])
          {
            cube(size=[fork_thickness, strafe_length, fork_width], center=false);
          }
          rotate([0, 90, 0])
          {
            cylinder(h=fork_thickness, r=fork_width/2);
          }
          rotate([0, -90, 0])
          {
            translate([0, 0, -$epsilon])
            cylinder(h=(bearing_dims[2] - fork_thickness/3 + $epsilon), r1=fork_width/2, r2=1.5* bearing_dims[0]);
          }
          translate([fork_thickness, 0, 0])
          rotate([0, 90, 0])
          translate([0, 0, -$epsilon])
          cylinder(h=(bearing_dims[2] - fork_thickness/3 + $epsilon), r1=fork_width/2, r2=1.5* bearing_dims[0]);
        }
        rotate([0, 90, 0])
        {
          union()
          {
            translate([0, 0, fork_thickness/3 - bearing_dims[2] - $epsilon])
            {
              cylinder(h=(2*bearing_dims[2] + fork_thickness/3 + 2*$epsilon), r=bearing_dims[0] - bearing_dims[3]);
              cylinder(h=(bearing_dims[2] + $epsilon), r=bearing_dims[0]);
            }
            translate([0, 0, 2*fork_thickness/3])
            {
              cylinder(h=(bearing_dims[2] + $epsilon), r=bearing_dims[0]);
            }
          }
        }
      }
    }    
  }
}
