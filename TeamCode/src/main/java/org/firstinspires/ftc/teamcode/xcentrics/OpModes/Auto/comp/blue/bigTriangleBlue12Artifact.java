package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue12paths;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;


@Configurable
@Autonomous(name = "Big Blue Triangle 9 Artifact",group = "blue")
public class bigTriangleBlue12Artifact extends LiveAutoBase {
    private Blue12paths paths;
    public  int pathState = 1;
    
    public static double maxSpeed = 1.5;
    
    public static double intakeSpeed = 0.5;
    @Override
    public void on_init() {
        Robot.isRed = false;
        paths = new Blue12paths(robot.follower);
        robot.follower.setStartingPose(paths.startPose);
    }

    @Override
    public void on_start() {
        robot.turret.turretOffset -= 10;
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        robot.intake.intake();
        update();
        robot.turret.update(this);
    }

    private void update(){
        switch(pathState){
            case 1:
                //follow score preload path
                followPath(paths.scorePreload,true);
                //aim the turret
                robot.turret.aim();
                //increment
                pathState++;
                break;

            case 2:
                if(notBusy()){
                    //score preload
                    halt(1.5);
                    robot.turret.shoot3();
                    //run intake
                    robot.intake.intake();
                    //stop aiming the turret
                    robot.turret.stopAim();
                    //follow pickup path slowly
                    followPath(paths.getFirstPattern, 1);
                    //increment
                    pathState = 4;
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
                    followPath(paths.pickupSecondPattern,1);
                    //increment
                    pathState++;
                    break;
                }

            case 7:
                if(notBusy()){
                    //turn off intake
                    //go to score position at full speed
                    followPath(paths.scoreSecondPattern,maxSpeed);
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
                    robot.intake.stopIntake();
                    pathState = 500;
                }
        }
    }
    private boolean notBusy(){
        return !robot.follower.isBusy();
    }
}
