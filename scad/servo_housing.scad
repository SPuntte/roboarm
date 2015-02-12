$fn = 50;

use <servo.scad>;

a = 45;
b = 41;
c = 39;
d = 20;
e = 53;
f = 30;


rotate([0, 90, 0]) servo(a, b, c, d, e, f);
/*
t = 3; // Thickness
translate([-(a + t), -(d/2 + 1 + t), -(d/2 + t)]) 
color("Blue") {
	cube([t, b + 2*t, d + 2*t]);
	translate([t, 0, t]) cube([f, t, d]);
	translate([t, b+t, t]) cube([f, t, d]);
}*/