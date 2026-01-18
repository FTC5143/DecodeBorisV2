package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.smallTriangleBlue;

public class Small9Blue extends LiveAutoBase {
    private smallTriangleBlue paths;
    private int p = 0;
    @Override
    public void on_init() {
        paths = new smallTriangleBlue(robot.follower);
    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {

    }

    private void update(){
        switch(p){
            case 0:
                robot.intake.intake().run();
        }
    }
}
