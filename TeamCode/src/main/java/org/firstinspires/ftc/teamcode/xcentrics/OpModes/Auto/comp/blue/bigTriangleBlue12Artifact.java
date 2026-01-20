package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue12paths;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;


@Configurable
@Autonomous(name = "Big Blue Beutafle Triangle NEINE Artifact",group = "blue")
public class bigTriangleBlue12Artifact extends LiveAutoBase {
    private Blue12paths paths;
    public  int pathState = 1;
    
    public static boolean b1;
    
    public static double launchWait = 0.7;
    
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

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        update();
        robot.turret.update(this);
    }

    private void update(){
        switch(pathState){
            case 1:
                //follow score preload path
                followPath(paths.scorePreload);
                //aim the turret
                robot.turret.aim();
                //increment
                pathState++;
                break;

            case 2:
                if(notBusy()){
                    //score preload
                    halt(1);
                        shoot3();
                        //run intake
                        robot.intake.intake();
                        //stop aiming the turret
                        robot.turret.stopAim();
                        //follow pickup path slowly
                        followPath(paths.getFirstPattern, intakeSpeed);
                        //increment
                        pathState = 4;
                    break;
                }

            case 3:
//                if(notBusy()){
//                    //stop intake
//                    robot.intake.stopIntake().run();
//                    //empty gate
//                    followPath(paths.emptyGate);
//                    //increment
//                    pathState++;
//                    break;
//                }

            case 4:
                if(notBusy()){
                    //aim turret
                    robot.turret.aim();
                    //go to score position at full speed
                    followPath(paths.scoreFirstPattern,maxSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 5:
                if(notBusy()) {
                    //score first pattern
                    shoot3();
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
                    followPath(paths.pickupSecondPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 7:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake();
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
                    shoot3();
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
    private void shoot3(){
        // 1
        halt(0.5);
        waitUntillSpin();
        robot.turret.launch();
        robot.update();
        //get next one ready
        robot.intake.setPower(-1);
        robot.update();
        halt(launchWait);
        //launch 2
        waitUntillSpin();
        robot.turret.launch();
        robot.update();

        //get next one ready
        halt(1.5);

        //launch third one

        robot.update();
        waitUntillSpin();
        robot.turret.launch();
        robot.intake.setPower(0);
        robot.update();
    }
    private void waitUntillSpin(){
        while((Math.abs(robot.turret.fly1.getVelocity() - 1600) <= 0) ){
            robot.update();
        }
    }
}
