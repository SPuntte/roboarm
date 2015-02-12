# roboarm
Aalto University ELEC-A4010 project work - see [Wiki page](https://wiki.aalto.fi/pages/viewpage.action?pageId=100214407).

## OpenSCAD parts

* elbow servo [housing](scad/elbow_servo_housing.scad), [axle](scad/elbow_servo_axle.scad), [lid](scad/elbow_servo_lid.scad) and [fork](scad/elbow_servo_fork.scad)

* shoulder-elbow [turss](scad/shoulder_truss.scad)

* elbow-wrist [turss](scad/elbow_truss.scad)

## Source code

* Arduino IDE [sketch](src/arm_control.ino) for the controller circurcuit
* mouse control with Inverse Kinematics [C++ source](src/arm_control.cpp)

## Misc.

* Wolfram Mathematica [notebook](ik.nb) of the IK solver
* gEDA [schematic](control.sch) for the MCU circuit