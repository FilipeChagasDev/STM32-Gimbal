$fn = 128;

/*cylinder*/
cy_di = 24; /*diameter*/
cy_thck = 4; /*thickness*/

/*rectangle*/
rec_h = 50;
rec_w = cy_di;
rec_thck = 4;  /*thickness*/

/*servo hole*/
hole_w = 14;
hole_h = 24;

difference()
{
    union()
    {
        difference()
        {
            cylinder(h=cy_thck, r1=cy_di/2, r2=cy_di/2, center=false);
        }

        difference()
        {
            translate([-rec_thck/2,-rec_w/2,0])
                cube(size=[rec_thck, rec_w, rec_h], center=false);

            translate([-rec_thck/2,-rec_w/2,0])
                translate([-1,5,rec_h-hole_h-10])
                    cube(size=[rec_thck+2, hole_w, hole_h], center=false);

            translate([-rec_thck/2,-rec_w/2,0])
                translate([-1,5,rec_h-hole_h-10])
                    translate([-2,hole_w/2 -1,-4])
                        cube(size=[rec_thck+4, 2, 6], center=false);

            translate([-rec_thck/2,-rec_w/2,0])
                translate([-1,5,rec_h-hole_h-10])
                    translate([-2,hole_w/2 -1,hole_h-1])
                        cube(size=[rec_thck+4, 2, 6], center=false);
        }
    }

    /*Clamp holes*/
    translate([5,3,-10])
        cube(size=[4,2,20]);

    translate([5,-3-2,-10])
        cube(size=[4,2,20]);

    translate([-9,3,-10])
        cube(size=[4,2,20]);

    translate([-9,-3-2,-10])
        cube(size=[4,2,20]);

    /*Horn hole*/
    translate([0,0,-2])
        cylinder(h=3, r1=4,r2=4,center=false);
}

/*
translate([0,0,-10])
    rotate(a=90,v=[0,1,0])
            rotate(a=90,v=[0,0,1])
                import(file = "horn.stl");
*/
