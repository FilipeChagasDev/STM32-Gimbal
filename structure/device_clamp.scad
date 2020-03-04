difference()
{
    union()
    {
        translate([-14, -50, 0])
            cube(size=[20,100,4], center=false);

        translate([10, -50, 0])
            cube(size=[3,100,8], center=false);
            
        rotate(a=90, v=[0,1,0]) 
        {
            /*Device handler*/
            translate([0,25,10])
                translate([-4,20,-10])
                    cube(size=[4,5,12], center=false);

            translate([0,-25,10])
                translate([-4,-25,-10])
                    cube(size=[4,5,12], center=false);
        }
    }

    rotate(a=90, v=[0,1,0])
    {
        /*Device handler holes*/
        translate([0,0,2])
            translate([-50,20-1,-4-1])
                cube(size=[100,5+2,4+2], center=false);

        translate([0,0,2])
            translate([-50,-25-1,-4-1])
                cube(size=[100,5+2,4+2], center=false);


        translate([0,0,2-2-4-2])
            translate([-50,20-1,-4-1])
                cube(size=[100,5+2,4+2], center=false);

        translate([0,0,2-2-4-2])
            translate([-50,-25-1,-4-1])
                cube(size=[100,5+2,4+2], center=false);

        /*central hole*/
        translate([0,0,2-2-4-2])
            translate([-40,-15,-4])
                cube(size=[100,30,12], center=false);

        /*central cut hole*/
        translate([0,0,2-2-4-2])
            translate([-40,-10,-4-5])
                cube(size=[100,20,12], center=false);

        /*side hole*/
        translate([0,0,2-2-4-2])
            translate([-40,-60,-4])
                cube(size=[100,30,12], center=false);

        /*side hole*/
        translate([0,0,2-2-4-2])
            translate([-40,30,-4])
                cube(size=[100,30,12], center=false);
    }
}
