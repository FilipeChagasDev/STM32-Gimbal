$fn = 128;

union()
{
    /*Device handler*/
    translate([-50,20,0])
        cube(size=[100,5,4], center=false);

    translate([-50,-25,0])
        cube(size=[100,5,4], center=false);

    /*Top*/
    translate([4,20,0])
        cube(size=[5,5,8], center=false);

    translate([4,-25,0])
        cube(size=[5,5,8], center=false);

    translate([-9,20,0])
        cube(size=[5,5,8], center=false);

    translate([-9,-25,0])
        cube(size=[5,5,8], center=false);

    /*Center*/
    difference()
    {
        translate([-12,-30,0])
            cube([24,60,4], center=false);

        /*Clamp holes*/
        rotate(a=90, v=[0, 0, 1])
        {
            translate([5,3,-10])
                cube(size=[4,2,20]);

            translate([5,-3-2,-10])
                cube(size=[4,2,20]);

            translate([-9,3,-10])
                cube(size=[4,2,20]);

            translate([-9,-3-2,-10])
                cube(size=[4,2,20]);
        }
    }
}
