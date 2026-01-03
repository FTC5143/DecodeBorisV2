package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.Comp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.LiveTeleopBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

@TeleOp(name = "TeleOp")

public class TeleopLive extends LiveTeleopBase {

    //small triangle points

    //robot pose
    private Pose redScorePose,blueScorePose;
    private boolean autoDrive = false,f1 = false;
    //triangle Checker
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
        Turret.autoAim = false;
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {

        //gamepad
        //driving
        if(!autoDrive) {
            robot.follower.setTeleOpDrive(
                    0 - gamepad1.left_stick_y,
                    0 - gamepad1.left_stick_x,
                    0 - gamepad1.right_stick_x
            );
            if (gamepad1.x) {
                if (robot.isRed()) {
                    robot.follower.setPose(new Pose(9, 9, Math.toRadians(0)));
                } else {
                    robot.follower.setPose(new Pose(135, 9, Math.toRadians(180)));
                }
            }
            if (gamepad1.left_bumper) {
                Robot.isRed = false;
            } else if (gamepad1.right_bumper) {
                Robot.isRed = true;
            }
        } else if(!f1){
            if(robot.isRed()) {
                robot.follower.followPath(robot.follower.pathBuilder()
                        .addPath(new BezierCurve(robot.follower::getPose,redScorePose))
                        .setLinearHeadingInterpolation(robot.follower.getPose().getHeading(), redScorePose.getHeading())
                        .build());
            } else {
                robot.follower.followPath(robot.follower.pathBuilder()
                        .addPath(new BezierCurve(robot.follower :: getPose, blueScorePose))
                        .setLinearHeadingInterpolation(robot.follower.getPose().getHeading(), blueScorePose.getHeading())
                        .build());
            }
            f1 = true;
        }

        if(gamepad1.left_bumper){
            autoDrive = true;
            f1 = false;
            halt(0.3);
        }
        if(gamepad1.right_bumper){
            robot.follower.breakFollowing();
            autoDrive = false;
        }
        //gamepad 1 controls

        if(gamepad2.dpad_up){
            Turret.a += 0.1;
            halt(0.1);
        } else if(gamepad2.dpad_down){
            Turret.a -= 0.1;
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

        //auto aim
        if(gamepad2.x && !Turret.autoAim){
            Turret.autoAim = true;
            halt(0.2);
        } else if(gamepad2.x && Turret.autoAim){
            Turret.autoAim = false;
            halt(0.2);
        }

        //turret offset
        if(gamepad2.dpad_left) {
           robot.turret.turretOffset +=1;
           halt(0.005);
        } else if(gamepad2.dpad_right){
            robot.turret.turretOffset -= 1;
            halt(0.005);
        }



    }
}
