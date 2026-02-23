package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.red;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.red.SmallTriangle;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.smallTriangleBlue;

@Autonomous(name = "Small Red Triangle 9 Artifact",group = "red")
public class Small9Red extends LiveAutoBase {


    SmallTriangle paths;
    private int p = 0;
    @Override
    public void on_init() {
        paths = new SmallTriangle( robot.follower);
        paths.mirrorPoints();
        robot.turret.far();
    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        update();
    }

    private void update(){
        switch(p) {
            case 0:
                followPath(paths.gotoscorepose);
                robot.turret.aim();
                p++;
                break;

            case 1:
                if(!robot.follower.isBusy()){
                    robot.turret.shoot3();
                    p++;
                    break;
                }

            case 2:
                if (!robot.follower.isBusy()){
                    followPath(paths.turntogetfirstpattern);
                    p++;
                    robot.turret.stopAim();
                    break;
                }

            case 3:
                if(!robot.follower.isBusy()){
                    robot.intake.intake();
                    followPath(paths.pickupfirstpattern,0.5);
                    p++;
                    break;
                }

            case 4:
                if(!robot.follower.isBusy()){
                    followPath(paths.scorefirstpattern,1);
                    robot.turret.aim();
                    p++;
                    break;
                }

            case 5:
                if(!robot.follower.isBusy()){
                    robot.turret.shoot3();
                    p++;
                    break;
                }

            case 6:
                if(!robot.follower.isBusy()){
                    followPath(paths.gotosecondpattern);
                    robot.turret.stopAim();
                    robot.intake.stopIntake();
                    p++;
                    break;
                }

            case 7:
                if(!robot.follower.isBusy()){
                    followPath(paths.pickupsecondpattern, 0.5);
                    robot.intake.intake();
                    p++;
                    break;
                }

            case 8:
                if(!robot.follower.isBusy()){
                    followPath(paths.scoresecondpattern,1);
                    robot.turret.aim();
                    p++;
                    break;
                }

            case 9:
                if(!robot.follower.isBusy()){
                    robot.turret.shoot3();
                    p++;
                    break;
                }

            case 10:
                followPath(paths.park);
                robot.turret.stopAim();
                robot.intake.stopIntake();
                p++;
                break;
        }
    }
}
