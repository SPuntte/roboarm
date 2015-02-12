module _servo_mounting_ear(a, b, c, d, e, f)
{
	difference() {
		cube(size=[d, (e-b)/2, 2]);
		union() {
			translate([5, 3, -0.5]) {
				cylinder(r=2, h=3);
				translate([-1, -3.5, 0]) cube([2, 2, 3]);
			}
			translate([15, 3, -0.5]) {
				cylinder(r=2, h=3);
				translate([-1, -3.5, 0]) cube([2, 2, 3]);
			}
		}
	}
}

module servo(a, b, c, d, e, f, x) {
	translate([-d/2, -(x), -a])
	union() {
		// Axle
		color("Gold") {
			translate([d/2, x, c]) {
				difference() {
					union() {
						cylinder(r=3, h=a-c-0.5);
						translate([0, 0, a-c-0.5]) cylinder(r1=3, r2=2.5, 0.5);
					}
					union() {
						translate([0, 0, 2]) cylinder(r=1, h=a-c-0.5);
						translate([0, 0, a-c-0.51]) cylinder(r1=1, r2=1.5, 0.52);
					}
				}
			}
		}
		// Body
		color("DarkGray") {
			translate([d/2, x, c-1]) cylinder(h=2, r1=d/3, r2=d/4);
			cube(size=[d, b, c-1]);
			// Mounting brackets
			translate([0, -(e-b)/2, f]) {
				_servo_mounting_ear(a, b, c, d, e, f);
			}
			translate([0, b+(e-b)/2, f])
			mirror ([0, 1, 0]) {
				_servo_mounting_ear(a, b, c, d, e, f);
			}
		}
	}
}