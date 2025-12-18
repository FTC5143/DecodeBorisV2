package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.xcentrics.util.Math.TriangleZoneChecker;

@TeleOp(name = "TeleOp")

public class TeleopLive extends LiveTeleopBase{
    private static boolean isInTriangle = false; //Are we within a triangle we can shoot in?
    private static boolean b1 = false;

    //small triangle points
    Pose sA, sB = new Pose(72,31), sC;

    //robot pose
    Pose robotPose;
    //triangle Checker
    TriangleZoneChecker triangleChecker = new TriangleZoneChecker();
    @Override
    public void on_init() {

    }

    @Override
    public void on_start() {
        robot.follower.startTeleOpDrive();
        if(!robot.isRed()) {
            robot.follower.setStartingPose(new Pose(63 ,9,Math.toRadians(180)));
        } else {
            robot.follower.setStartingPose(new Pose(81,9,Math.toRadians(0)));
        }
        robot.turret.autoAim = false;
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {

        //check to see if the robot is within a triangle and do the apropreate thing
        robotPose = robot.follower.getPose();
        //gamepad controls
                    robot.follower.setTeleOpDrive(
                -gamepad1.right_stick_x ,
                -gamepad1.left_stick_x,
                           - gamepad1.left_stick_y,
                            true
                            // Robot Centric
        );
                    if(gamepad1.x){
                        if(robot.isRed()){
                            robot.follower.setPose(new Pose(9,9,Math.toRadians(0)));
                        } else {
                            robot.follower.setPose(new Pose(135, 9, Math.toRadians(180)));
                        }
                    }
                    if(gamepad1.left_bumper){
                        robot.isRed = false;
                    } else if (gamepad1.right_bumper){
                        robot.isRed = true;
                    }
        //gamepad 1 controls

        if(gamepad2.dpad_up){
            robot.turret.a += 0.1;
            halt(0.1);
        } else if(gamepad2.dpad_down){
            robot.turret.a -= 0.1;
            halt(0.1);
        }

        if(gamepad2.left_bumper){
            robot.intake.setPower(-1);
        } else if(gamepad2.right_bumper){
            robot.intake.setPower(1);
        } else {
            robot.intake.setPower(0);
        }

        //launch ball(s)

        if(gamepad2.a){
            robot.turret.launch();
        }
        if(gamepad2.x && !robot.turret.autoAim){
            robot.turret.autoAim = true;
        } else if(gamepad2.x && robot.turret.autoAim){
            robot.turret.autoAim = false;
        }

        if(gamepad2.dpad_left) {
           robot.turret.turretOffset +=1;
           halt(0.005);
        } else if(gamepad2.dpad_right){
            robot.turret.turretOffset -= 1;
            halt(0.005);
        }


    }
    public void shoot1ball(){
        robot.intake.setPower(1);
        robot.turret.launch();
        robot.intake.setPower(0);
        robot.turret.resetKicker();
        halt(0.5);
    }
}
