package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Camera;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Intake;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Spindexer;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;

public class LiveRobot extends Robot{
    public Follower follower;
    public Intake intake;
    public Spindexer spindexer;
    public Turret turret;
    public Camera camera;
    {
        name = "BORISV2";
    }
    public LiveRobot(LinearOpMode opMode){
        super(opMode);
        follower    = Constants.createFollower(hwmap);
        intake      = new Intake(this);
        spindexer   = new Spindexer(this);
        turret      = new Turret(this,this);
    }

    @Override
    public void update(){
        super.update();
        follower.update();
    }

    @Override
    public void updateTelemetry(){
        super.updateTelemetry();
    }
}
