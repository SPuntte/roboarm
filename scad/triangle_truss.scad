module truss_edge_beam(beam_width, edge_extrude, total_length)
{
  edge_width = beam_width * tan(60);
  edge_bevel = beam_width / (2 * cos(60));
  bevel_x = edge_bevel - (1 + $epsilon) * beam_width * cos(60) / 2;
  bevel_y = (1 + $epsilon) * beam_width * sin(60) / 2;
  linear_extrude(height = total_length)
  {
    polygon([[edge_bevel, 0], [edge_width + edge_extrude, 0],
        [edge_width + edge_extrude, beam_width + tan(30) * edge_extrude],
        [edge_width, beam_width], [bevel_x, bevel_y]]);
  }
  echo("Main bean height:", beam_width + tan(30) * edge_extrude, "mm");
}

module truss_main_beams(beam_width, edge_extrude, inner_width, total_length)
{
  edge_width = beam_width * tan(60);
  union()
  {
    truss_edge_beam(beam_width, edge_extrude, total_length);
    translate([2*(edge_width+edge_extrude) + inner_width, 0, 0])
    {
      mirror([1, 0, 0])
      {
        truss_edge_beam(beam_width, edge_extrude, total_length);
      }
    }
  }
}

module truss_end_triangle(beam_width, edge_extrude, inner_width, screw_d, screw_position)
{
  difference()
  {
    linear_extrude(height = beam_width)
    {
      polygon([[-$epsilon, 0], [inner_width + $epsilon, 0], [inner_width + $epsilon, beam_width],
               [inner_width + edge_extrude, beam_width],
               [inner_width/2, beam_width + sqrt(3)*(inner_width + 2*edge_extrude)/6 + $epsilon], 
               [-edge_extrude, beam_width], [-$epsilon, beam_width]]);
    }

    if (screw_position == 0)
    {
      translate([inner_width - (2 + screw_d/2), 2 + screw_d/2, -$epsilon])
      {
        cylinder(h=beam_width + 2*$epsilon, r=(screw_d/2 + $padding), center=false);
      }
    }
    else
    {
      translate([inner_width/2, 2 + screw_d/2, -$epsilon])
      {
        cylinder(h=beam_width + 2*$epsilon, r=(screw_d/2 + $padding), center=false);
      }
    }
  }
}

module truss_sweep(beam_width, edge_extrude, inner_width, num_inner_beams, inner_beam_angle, top)
{
  inner_beam_length = inner_width / tan(inner_beam_angle);
  inner_beam_offset = beam_width / sin(inner_beam_angle);
  total_length = beam_width + num_inner_beams*inner_beam_length +
                 inner_beam_offset + (num_inner_beams - 1)*inner_beam_offset/3;

  for (i = [0 : 1.0: num_inner_beams - 1])
  {
    translate([0, beam_width, beam_width/2 + i*(inner_beam_length+inner_beam_offset/3)])
    {
      union()
      {
        if (top == 1 && i > 0)
        {
          translate([-$epsilon, -beam_width, inner_beam_offset/3 - beam_width/2])
          cube([inner_width + 2*$epsilon, beam_width, beam_width]);
        }
        if (i % 2 == 0)
        {
          rotate([90, 0, 0])
          {
            linear_extrude(height=beam_width)
            {
              polygon([[-$epsilon, 0], [inner_width + $epsilon, inner_beam_length],
                       [inner_width + $epsilon, inner_beam_length + inner_beam_offset],
                       [-$epsilon, inner_beam_offset]]);
            }
          }
        }
        else
        {
          rotate([90, 0, 0])
          {
            linear_extrude(height=beam_width)
            {
              polygon([[inner_width + $epsilon, 0], [-$epsilon, inner_beam_length],
                       [-$epsilon, inner_beam_length + inner_beam_offset],
                       [inner_width + $epsilon, inner_beam_offset]]);
            }
          }
        }
      }
    }
  }
}

module triangle_truss_face(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position, top)
{
  edge_width = beam_width * tan(60);
  inner_beam_length = inner_width / tan(inner_beam_angle);
  inner_beam_offset = beam_width / sin(inner_beam_angle);

  total_length = beam_width + num_inner_beams*inner_beam_length +
                 inner_beam_offset + (num_inner_beams - 1)*inner_beam_offset/3;
  union()
  {
    truss_main_beams(beam_width, edge_extrude, inner_width, total_length);

    translate([edge_width + edge_extrude, 0, 0])
    {
      union()
      {
        truss_end_triangle(beam_width, edge_extrude, inner_width, screw_d, screw_position);
        translate([0, 0, total_length - beam_width])
        {
          truss_end_triangle(beam_width, edge_extrude, inner_width, screw_d, screw_position);
        }

        truss_sweep(beam_width, edge_extrude, inner_width, num_inner_beams, inner_beam_angle, top);
      }
    }
  }
}

module triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position)
{
  edge_width = beam_width * tan(60);
  total_width = 2*(edge_width+edge_extrude) + inner_width;

  union()
  {
    triangle_truss_face(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position, 1);

    translate([total_width, 0, 0])
    {
      rotate([0, 0, 120])
      {
        triangle_truss_face(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position, 1);
      }
    }

    rotate([0, 0, -120])
    {
      mirror([1, 0, 0])
      translate([0, 0, 0])
      {
        triangle_truss_face(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position, 1);
      }
    }
  }
}

module triangle_truss_dims(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width)
{
  inner_beam_length = inner_width / tan(inner_beam_angle);
  inner_beam_offset = beam_width / sin(inner_beam_angle);

  total_length = beam_width + num_inner_beams*inner_beam_length +
                 inner_beam_offset + (num_inner_beams - 1)*inner_beam_offset/3;

  edge_width = beam_width * tan(60);
  edge_bevel = beam_width / (2 * cos(60));
  bevel_x = edge_bevel - beam_width * cos(60);

  total_width = 2*(edge_width - edge_bevel + bevel_x) +
                2*edge_extrude + inner_width;

  echo("Truss length: ", total_length, " mm");
  echo("Truss width: ", total_width, " mm");
}
