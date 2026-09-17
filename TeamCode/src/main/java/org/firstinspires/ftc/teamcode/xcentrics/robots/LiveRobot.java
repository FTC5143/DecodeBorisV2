package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Shooter;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.intake;


import java.util.concurrent.TimeUnit;


public class LiveRobot extends Robot{
    public Follower follower;
    public Shooter shooter;
    public intake intake;

    public static Pose lastPose = new Pose(0,0,Math.toRadians(0));
    {
        name = "BUMBULBEE";
    }

    public LiveRobot(LinearOpMode opMode) {
        super(opMode);
        follower    = Constants.createFollower(hwmap);
        intake = new intake(this);
        shooter = new Shooter(this);
    }


    @Override
    public void update(){
        super.update();
        follower.update();
       lastPose = follower.getPose();
    }
    public void startup(){
        isRed = true;
    }
    @Override
    public  void updateTelemetry(){
        super.updateTelemetry();
    }
    public void setLastPose(Pose p){
        lastPose = p;
    }
    public Pose getLastPose(){
        return lastPose;
    }

    /**
     * Get current robot pose from vision or follower
     */
    public Pose getRobotPose() {
        return follower.getPose();
    }
    private volatile long startTime = 0; // in nanoseconds
    public void halt(double seconds) {
        resetRuntime();
        while (getRuntime() < seconds) {
            update();
        }
    }
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }
    public void resetRuntime() {
        startTime = System.nanoTime();
    }
}
