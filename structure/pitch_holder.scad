$fn = 128;

cylinder_height = 14;
cylinder_diameter = 27-1;

rec1_x_len = 25+29;
rec1_y_len = cylinder_diameter;
rec1_thickness = 4;

rec2_thickness = 4;
rec2_h = 60;
rec2_w = cylinder_diameter;

servo_hole_h = 24;
servo_hole_w = 14;

little_hole_w = 2;
little_hole_h = 5;

union()
{
    cylinder(h = cylinder_height, r1 = cylinder_diameter/2, r2 = cylinder_diameter/2, center = false);

    /*Rec1*/
    translate([-rec1_x_len,-rec1_y_len/2, cylinder_height-rec1_thickness])
        cube(size=[rec1_x_len,rec1_y_len,rec1_thickness], center=false);

    difference()
    {
        /*Rec2*/
        translate([-rec1_x_len,-rec1_y_len/2,cylinder_height])
        cube(size=[rec2_thickness,rec2_w,rec2_h], center=false);

        /*Servo hole*/
        translate([-rec1_x_len-1,-cylinder_diameter/2 + 6.5, cylinder_height+rec2_h-servo_hole_h-10])
        cube(size=[rec2_thickness+2,servo_hole_w,servo_hole_h], center=false);

        /*Top little hole*/
        translate([-rec1_x_len-1,-cylinder_diameter/2 + 6.5, cylinder_height+rec2_h-servo_hole_h-10])
        translate([-1, servo_hole_w/2 - little_hole_w/2, servo_hole_h-1])
        cube(size=[7, little_hole_w, little_hole_h+2], center=false);

        /*Bottom little hole*/
        translate([-rec1_x_len-1,-cylinder_diameter/2 + 6.5, cylinder_height+rec2_h-servo_hole_h-10])
        translate([-1, servo_hole_w/2 - little_hole_w/2, -5])
        cube(size=[7, little_hole_w, little_hole_h+2], center=false);
    }
}
