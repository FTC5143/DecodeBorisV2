package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.smallTriangleBlue;
@Autonomous(name = "Small Blue Beutafle Triangle NEINE Artifact",group = "blue")
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
        update();
    }

    private void update(){
        switch(p){
            case 0:
                robot.turret.aim();
                halt(1);
                robot.intake.intake();
                robot.turret.shoot3();
                halt(1);
                p++;
                break;

            case 1:
                robot.turret.stopAim();
                robot.intake.stopIntake();
                followPath(paths.goToFirstPattern);
                p++;
                break;

            case 2:
                if(!robot.follower.isBusy()){
                    robot.intake.intake();
                    followPath(paths.getFirstPattern,0.5);
                    p++;
                    break;
                }

            case 3:
                if(!robot.follower.isBusy()){
                    robot.intake.stopIntake();
                    robot.turret.aim();
                    followPath(paths.scoreFirstPattern,1);
                    p++;
                    break;
                }

            case 4:
                if(!robot.follower.isBusy()){
                    robot.intake.intake();
                    robot.turret.shoot3();
                    halt(1);
                    p++;
                    break;
                }

            case 5:
                robot.intake.stopIntake();
                robot.turret.stopAim();
                followPath(paths.goToSecondPattern);
                p++;
                break;

            case 6:
                if(!robot.follower.isBusy()){
                    robot.intake.intake();
                    followPath(paths.getSecondPattern,0.5);
                    p++;
                    break;
                }

            case 7:
                if(!robot.follower.isBusy()){
                    robot.intake.stopIntake();
                    robot.turret.aim();
                    followPath(paths.scoreSecondPattern,1);
                    p++;
                    break;
                }

            case 8:
                if(!robot.follower.isBusy()){
                    robot.intake.intake();
                    robot.turret.shoot3();
                    halt(1);
                    p++;
                    break;
                }

            case 9:
                robot.turret.stopAim();
                robot.intake.stopIntake();
                followPath(paths.park);
                p++;
                break;

            case 10:
                robot.follower.holdPoint(new Pose(38.5,33.5,Math.toRadians(90)));
        }
    }
}
