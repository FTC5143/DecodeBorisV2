package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.xcentrics.util.Math.TriangleZoneChecker;
import org.firstinspires.ftc.teamcode.xcentrics.util.Math.TriangleZoneChecker;

public class TeleopLive extends LiveTeleopBase{
    private static boolean isInTriangle = false; //Are we within a triangle we can shoot in?

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

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {

        //check to see if the robot is within a triangle and do the apropreate thing
        robotPose = robot.follower.getPose();
        //gamepad controls

        //gamepad 1 controls
    }
}
