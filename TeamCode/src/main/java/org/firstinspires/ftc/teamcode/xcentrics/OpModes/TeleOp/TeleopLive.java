package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.xcentrics.components.live.Camera;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.util.Math.TriangleZoneChecker;

@TeleOp(name = "TeleOp")

public class TeleopLive extends LiveTeleopBase{

    //small triangle points
    Pose sA, sB = new Pose(72,31), sC;

    //robot pose
    Pose robotPose,redScorePose,blueScorePose;
    private boolean autoDrive = false,f1 = false;
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
        //gamepad
        //driving
        if(!autoDrive) {
            robot.follower.setTeleOpDrive(
                    0 + gamepad1.left_stick_y,
                    0 + gamepad1.left_stick_x,
                    0 + gamepad1.right_stick_x
            );
            if (gamepad1.x) {
                if (robot.isRed()) {
                    robot.follower.setPose(new Pose(9, 9, Math.toRadians(0)));
                } else {
                    robot.follower.setPose(new Pose(135, 9, Math.toRadians(180)));
                }
            }
            if (gamepad1.left_bumper) {
                robot.isRed = false;
            } else if (gamepad1.right_bumper) {
                robot.isRed = true;
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
            robot.turret.a += 0.1;
            halt(0.1);
        } else if(gamepad2.dpad_down){
            robot.turret.a -= 0.1;
            halt(0.1);
        }

        if(gamepad2.left_bumper){
            robot.intake.setPower(-1);
            robot.spin.intakeOne();
        } else if(gamepad2.right_bumper){
            robot.intake.setPower(1);
        } else {
            robot.intake.setPower(0);
        }

        //launch ball(s)

        if(gamepad2.a){
            robot.spin.shoot();
        }

        //auto aim
        if(gamepad2.x && !robot.turret.autoAim){
            robot.turret.autoAim = true;
        } else if(gamepad2.x && robot.turret.autoAim){
            robot.turret.autoAim = false;
        }

        //turret offset
        if(gamepad2.dpad_left) {
           robot.turret.turretOffset +=1;
           halt(0.005);
        } else if(gamepad2.dpad_right){
            robot.turret.turretOffset -= 1;
            halt(0.005);
        }

        //shoot pattern
        if(gamepad2.a && gamepad2.left_bumper){
            robot.spin.ejectPattern();
        }


    }
}
