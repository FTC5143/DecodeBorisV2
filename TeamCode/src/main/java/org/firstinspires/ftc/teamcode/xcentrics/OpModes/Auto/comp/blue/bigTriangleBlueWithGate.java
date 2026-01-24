package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue9WithGatePaths;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Red9WithGatePaths;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

public class bigTriangleBlueWithGate extends LiveAutoBase {
    private Blue9WithGatePaths paths;
    private int pathState = 0;
    @Override
    public void on_init() {
        Robot.isRed = true;
        paths = new Blue9WithGatePaths(robot.follower);
        robot.follower.setStartingPose(paths.startPose);
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
        switch (pathState){
            case 0:
                followPath(paths.scorePreload);
                robot.turret.aim();
                pathState++;
                break;

            case 1:
                if(b()){
                    robot.turret.shoot3();
                    followPath(paths.getFirstPattern,0.5);
                    robot.turret.stopAim();
                    robot.intake.intake();
                    pathState++;
                    break;
                }

            case 2:
                if(b()){
                    robot.intake.stopIntake();
                    followPath(paths.emptyGate);
                    pathState++;
                    break;
                }

            case 3:
                if(b()){
                    robot.turret.aim();
                    followPath(paths.scoreFirstPattern,1);
                    pathState++;
                    break;
                }

            case 4:
                if(b()){
                    robot.turret.shoot3();
                    followPath(paths.goToSecondPattern);
                    robot.turret.stopAim();
                    pathState++;
                    break;
                }

            case 5:
                if(b()){
                    robot.intake.intake();
                    followPath(paths.getSecondPattern,0.5);
                    pathState++;
                    break;
                }

            case 6:
                if(b()){
                    robot.turret.aim();
                    robot.intake.stopIntake();
                    followPath(paths.scoreSecondPattern);
                    pathState++;
                    break;
                }

            case 7:
                if(b()){
                    robot.turret.shoot3();
                    followPath(paths.park);
                    robot.turret.stopAim();
                    pathState++;
                    break;
                }
        }
    }
    private boolean b(){
        return !robot.follower.isBusy();
    }
}
