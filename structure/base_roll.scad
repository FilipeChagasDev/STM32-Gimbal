$fn = 128;

h = 30;
w = 24;
t = 4;

difference()
{
    union()
    {
        translate([-12,-2,0])
            cube([w, t, h], center = false);

        translate([-2,-12,0]) /*H Clamp holes*/
            cube([t, w, h], center = false);

        translate([0, 0,0])
            cylinder(h = t, r = w/2, center = false);
    }

    /*V Clamp holes*/
    translate([5,3,-10])
        cube(size=[4,2,20]);

    translate([5,-3-2,-10])
        cube(size=[4,2,20]);

    translate([-9,3,-10])
        cube(size=[4,2,20]);

    translate([-9,-3-2,-10])
        cube(size=[4,2,20]);

    /*H Clamp holes*/
    translate([5,-3-2,4])
        cube([4,10,2], center=false);

    translate([-9,-3-2,4])
        cube([4,10,2], center=false);

    rotate(a=90, v=[0,0,1])
    {
        translate([5,-3-2,25])
            cube([4,10,2], center=false);

        translate([-9,-3-2,25])
            cube([4,10,2], center=false);
    }

    /*Horn hole*/
    translate([0,0,-2])
        cylinder(h=3, r1=4,r2=4,center=false);
}

