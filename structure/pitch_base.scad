/*cylinder*/
cy_di = 24; /*diameter*/
cy_thck = 4; /*thickness*/

/*rectangle*/
rec_h = 45;
rec_w = cy_di;
rec_thck = 4;  /*thickness*/

/*servo hole*/
hole_w = 14;
hole_h = 24;

union()
{
    cylinder(h=cy_thck, r1=cy_di/2, r2=cy_di/2, center=false);

    difference()
    {
        translate([-rec_thck/2,-rec_w/2,0])
            cube(size=[rec_thck, rec_w, rec_h], center=false);

        translate([-rec_thck/2,-rec_w/2,0])
        {
            translate([-1,5,rec_h-hole_h-5])
                cube(size=[rec_thck+2, hole_w, hole_h], center=false);
        }
    }
}
