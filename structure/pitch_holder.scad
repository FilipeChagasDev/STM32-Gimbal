cylinder_height = 14;
cylinder_diameter = 27;

rec1_x_len = 25;
rec1_y_len = cylinder_diameter;
rec1_thickness = 4;

rec2_thickness = 4;
rec2_h = 55;
rec2_w = cylinder_diameter;

servo_hole_h = 24;
servo_hole_w = 14;

union()
{
    cylinder(h = cylinder_height, r1 = cylinder_diameter/2, r2 = cylinder_diameter/2, center = false);
    
    translate([-rec1_x_len,-rec1_y_len/2, cylinder_height-rec1_thickness])
        cube(size=[rec1_x_len,rec1_y_len,rec1_thickness], center=false);

    difference()
    {
        translate([-rec1_x_len,-rec1_y_len/2,cylinder_height]) 
            cube(size=[rec2_thickness,rec2_w,rec2_h], center=false);
        
        translate([-rec1_x_len-1,-cylinder_diameter/2 + 6.5, cylinder_height+rec2_h-servo_hole_h-5])
            cube(size=[rec2_thickness+2,servo_hole_w,servo_hole_h], center=false);
    }
    
}