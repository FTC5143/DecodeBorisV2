package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue9WithGatePaths;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
@Autonomous(name = "big blue triangle 9 with gate (Delta)", group = "blue")
public class bigTriangleBlueWithGate extends LiveAutoBase {
    private Blue9WithGatePaths paths;
    private int pathState = 1;
    @Override
    public void on_init() {
        Robot.isRed = false;
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
            case 1:
                robot.turret.aim();
                //follow score preload path
                followPath(paths.scorePreload);
                //aim the turret

                //increment
                pathState++;
                break;

            case 2:
                if(notBusy()){
                    //score preload
                    halt(1);
                    robot.turret.shoot3();
                    //run intake
                    robot.intake.intake();
                    //stop aiming the turret
                    robot.turret.stopAim();
                    //follow pickup path slowly
                    followPath(paths.getFirstPattern, 0.4);
                    robot.intake.intake();
                    //increment
                    pathState = 4;
                    break;
                }

            case 3:
                if(notBusy()){
                    //stop intake
                    robot.intake.stopIntake();
                    //empty gate
                    followPath(paths.emptyGate);
                    //increment
                    pathState++;
                    break;
                }

            case 4:
                if(notBusy()){
                    //aim turret
                    robot.turret.aim();
                    //go to score position at full speed
                    followPath(paths.scoreFirstPattern,1);
                    //increment
                    pathState++;
                    break;
                }

            case 5:
                if(notBusy()) {
                    //score first pattern
                    robot.turret.shoot3();
                    //stop aiming
                    robot.turret.stopAim();

                    //go to second pattern
                    followPath(paths.goToSecondPattern);
                    //increment
                    pathState++;
                    break;
                }

            case 6:
                if(notBusy()){
                    //turn on intake
                    robot.intake.intake();
                    //pickup second pattern slowly
                    followPath(paths.getSecondPattern,0.4);
                    //increment
                    pathState++;
                    break;
                }

            case 7:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake();
                    //go to score position at full speed
                    followPath(paths.scoreSecondPattern,1);
                    //aim turret
                    robot.turret.aim();
                    //increment
                    pathState = 11;
                    break;
                }
            case 11:
                if(notBusy()){
                    //score third pattern
                    robot.turret.shoot3();
                    //stop aiming the turret
                    robot.turret.stopAim();

                    //go to the park position
                    followPath(paths.park,1);
                    pathState = 500;
                }
        }
    }
    private boolean notBusy(){
        return !robot.follower.isBusy();
    }
}
