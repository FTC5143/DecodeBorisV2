package org.firstinspires.ftc.teamcode.xcentrics.OpModes;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.basic.basic;
import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue.Small9Blue;
import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.red.bigTriangleRed12artifact;

public class classes {


    private final Small9Blue s = new Small9Blue();

    public classes(){
        basic b = new basic();
        b.runOpMode();
        s.runOpMode();
        bigTriangleRed12artifact d = new bigTriangleRed12artifact();
        d.on_init();
        classes t = new classes();
        t.s.runOpMode();
    }
    public void p(){
        s.runOpMode();
    }


}
