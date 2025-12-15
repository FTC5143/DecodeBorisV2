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
                -gamepad1.left_stick_y ,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
        //gamepad 1 controls

        if(gamepad2.dpad_up){
            robot.intake.setPower(-0.5);
        } else if(gamepad2.dpad_down){
            robot.intake.setPower(0.5);
        } else {
            robot.intake.setPower(0);
        }

        if(gamepad2.dpad_left && !b1){
            robot.turret.manualTurret();
        } else if(gamepad2.dpad_left && b1){
            robot.turret.autoAim = true;
        }

        //launch ball(s)
        if(gamepad2.x) {
            robot.turret.launch();
        }


    }
}
