package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Camera;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Intake;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Spin;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Spindexer;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
import org.json.JSONException;
import org.json.JSONObject;
public class LiveRobot extends Robot{
    public  Follower follower;
    public  Intake intake;
    public  Spindexer spindexer;
    public  Turret turret;
    public  Camera camera;
    public  Spin spin;
    public static JSONObject robotJson;
    public static Pose lastPose;
    {
        name = "BORISV2";
    }
    public LiveRobot(LinearOpMode opMode){
        super(opMode);
        follower    = Constants.createFollower(hwmap);
        intake      = new Intake(this,this);
        //spindexer   = new Spindexer(this);
        turret      = new Turret(this,this);
        spin        = new Spin(this,this);
    }

    @Override
    public void update(){
        super.update();
        follower.update();
    }
    public void startup(){
        try {
            isRed = robotJson.getBoolean("isRed");
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
    }


    @Override
    public  void updateTelemetry(){
        super.updateTelemetry();
    }
    public void switchAlliance(boolean b) throws JSONException {
        robotJson.put("isRed",b);
    }
    public void setPattern(Camera.Pattern pattern) throws JSONException{
        robotJson.put("pattern",pattern);
    }
    public Camera.Pattern getPattern(){
        try {
            return (Camera.Pattern) robotJson.get("pattern");
        } catch (JSONException e){
            return Camera.Pattern.PPG;
        }
    }
    public void setLastPose(Pose p){
        lastPose = p;
    }
    public Pose getLastPose(){
        return lastPose;
    }
}
