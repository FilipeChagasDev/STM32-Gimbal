structure_color = [0.3,0.3,0.3];
servo_color = [1,0,0];
horn_color = [0.7,0.7,0.7];

pitch_rotation = 0;
roll_rotation = 0;

/* -- PITCH HOLDER -- */
//pitch holder
color(structure_color) import("stl/pitch_holder.stl");

//servo
color(servo_color)
    translate([-36.8,-6.2,35])
        rotate(a=90, v=[0,1,0])
            rotate(a=90, v=[1,0,0])
                rotate(a=180, v=[0,1,0])
                    import("servo/mg90s.stl");



/* -- PITCH AXIS -- */
translate([-6,0,57])
rotate(a=pitch_rotation, v=[1,0,0])
{
    //horn
    color(horn_color)
        rotate(a=-90, v=[0,0,1])
            rotate(a=90, v=[0,1,0])
                import("servo/horn.stl");

    //pitch base
    color(structure_color)
        translate([1,0,0])
            rotate(a=90, v=[0,1,0])
                rotate(a=90, v=[0,0,1])
                    import("stl/pitch_base.stl");

    //servo
    color(servo_color)
        translate([12,-14,-6.2])
            import("servo/mg90s.stl");


    /* -- ROLL AXIS -- */
    translate([34.5,17,0])
    rotate(a=roll_rotation,v=[0,1,0])
    {
        //horn
        color(horn_color)
            rotate(a=-90, v=[1,0,0])
                rotate(a=-90, v=[0,0,1])
                    rotate(a=90, v=[1,0,0])
                        import("servo/horn.stl");

        //roll base
        color(structure_color)
            translate([0,1,0])
                rotate(a=90, v=[0,1,0])
                    rotate(a=3*90, v=[1,0,0])
                        import("stl/roll_base.stl");
        
        //device holder
        color(structure_color)
            translate([0,31,0])
                rotate(a=90,v=[0,1,0])
                    rotate(a=-90,v=[1,0,0])
                        import("stl/device_holder.stl");

        //device holder 2
        color(structure_color)
            translate([0,39,0])
                rotate(a=90,v=[0,1,0])
                    rotate(a=-90,v=[1,0,0])
                        import("stl/device_holder2.stl");
    }
}
