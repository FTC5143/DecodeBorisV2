package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue12paths;
@Configurable
@TeleOp(name = "Blue Big Triangle (12 Artifacts)")
public class bigTriangleBlue12Artifact extends LiveAutoBase {
    private final Blue12paths paths = new Blue12paths(robot.follower);
    private static int pathState = 0;
    public static double launchWait = 0.3;
    public static double maxSpeed = 1;
    public static double intakeSpeed = 0.5;
    @Override
    public void on_init() {
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
        robot.addData("pathState:",pathState);
    }

    private void update(){
        switch(pathState){
            case 1:
                //follow score preload path
                followPath(paths.scorePreload);
                //aim the turret
                robot.turret.aim().run();
                //increment
                pathState++;
                break;

            case 2:
                if(notBusy()){
                    //score preload
                    shoot3();
                    //run intake
                    robot.intake.intake().run();
                    //stop aiming the turret
                    robot.turret.stopAim().run();
                    //follow pickup path slowly
                    followPath(paths.getFirstPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 3:
                if(notBusy()){
                    //stop intake
                    robot.intake.stopIntake().run();
                    //empty gate
                    followPath(paths.emptyGate);
                    //increment
                    pathState++;
                    break;
                }

            case 4:
                if(notBusy()){
                    //aim turret
                    robot.turret.aim().run();
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
                    robot.turret.stopAim().run();

                    //go to second pattern
                    followPath(paths.goToSecondPattern);
                    //increment
                    pathState++;
                    break;
                }

            case 6:
                if(notBusy()){
                    //turn on intake
                    robot.intake.intake().run();
                    //pickup second pattern slowly
                    followPath(paths.pickupSecondPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 7:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake().run();
                    //go to score position at full speed
                    followPath(paths.scoreSecondPattern,maxSpeed);
                    //aim turret
                    robot.turret.aim().run();
                    //increment
                    pathState++;
                    break;
                }
            case 8:
                if(notBusy()){
                    //score second pattern
                    shoot3();
                    //stop aiming the turret
                    robot.turret.stopAim();

                    //go to the third pattern
                    followPath(paths.goToThirdPattern);
                    //increment
                    pathState++;
                    break;
                }

            case 9:
                if(notBusy()){
                    //turn on intake
                    robot.intake.intake().run();
                    //pick up balls slowly
                    followPath(paths.pickUpThirdPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 10:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake().run();
                    //go to scoring position at full speed
                    followPath(paths.scoreThirdPattern,maxSpeed);
                    //aim the turret
                    robot.turret.aim().run();
                    //increment
                    pathState++;
                    break;
                }

            case 11:
                if(notBusy()){
                    //score third pattern
                    shoot3();
                    //stop aiming the turret
                    robot.turret.stopAim().run();

                    //go to the park position
                    followPath(paths.park,0.7);
                }
        }
    }
    private boolean notBusy(){
        return !robot.follower.isBusy();
    }
    private void shoot3(){
        // 1
        robot.turret.launch();
        halt(launchWait);
        //get next one ready
        robot.intake.intake().run();
        halt(launchWait);
        //launch 2
        robot.intake.stopIntake().run();
        robot.turret.launch();
        halt(launchWait);
        //get next one ready
        robot.intake.intake().run();
        halt(launchWait);

        //launch third one
        robot.intake.stopIntake().run();
        robot.turret.launch();
        halt(launchWait);
    }
}
