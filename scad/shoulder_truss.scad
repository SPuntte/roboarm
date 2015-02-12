$fn = 50;
$epsilon = 0.05;
$padding = 0.2;

use <triangle_truss.scad>

// triangle_truss(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width);

beam_width = 5;
edge_extrude = 5;
num_inner_beams = 4;
inner_beam_angle = 45;
inner_width = 30;
screw_d = 3;
screw_position = 1;

triangle_truss_face(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width, screw_d, screw_position, 1);
triangle_truss_dims(beam_width, edge_extrude, num_inner_beams, inner_beam_angle, inner_width);
