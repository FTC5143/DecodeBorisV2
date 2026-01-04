package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Intake;

import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
public class LiveRobot extends Robot{
    public  Follower follower;
    public  Intake intake;
    public  Turret turret;
    public static Pose lastPose = new Pose(0,0,Math.toRadians(0));
    {
        name = "Daniel"; //from the writing on the wall (daniel chapter 5)
    }
    public LiveRobot(LinearOpMode opMode) {
        super(opMode);
        follower    = Constants.createFollower(hwmap);
        intake      = new Intake(this);
        turret      = new Turret(this);
    }

    @Override
    public void update(){
        super.update();
        follower.update();
       setLastPose(follower.getPose());
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
}
