module bearing(dims)
{
  outer_r = dims[0];
  inner_r = dims[1];
  height  = dims[2];
  outer_w = dims[3];
  inner_w = dims[4];

  color("DarkGray")
  {
    union()
    {
      difference()
      {
        cylinder(h = height, r = outer_r);
        translate([0, 0, -$epsilon])
        {
          cylinder(h = height + 2*$epsilon, r = outer_r - outer_w);
        }
      }
      difference()
      {
        cylinder(h = height, r = inner_r + inner_w);
        translate([0, 0, -$epsilon])
        {
          cylinder(h = height + 2*$epsilon, r = inner_r);
        }
      }
    }
  }
}
