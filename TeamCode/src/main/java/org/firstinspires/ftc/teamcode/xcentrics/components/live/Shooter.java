package org.firstinspires.ftc.teamcode.xcentrics.components.live;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;

class LauncherConfig{
    public static int TARGET_VELO = 1250;
    public static PIDFCoefficients flyPidCoef = new PIDFCoefficients(40,0,0,12.5);
    public static int LAUNCHER_MIN_VELO = 1200;
}
public class Shooter extends Component {

    public DcMotorEx fly;
    private CRServoQUS gate;
    private int fly1 = 0,gate1 = 0;
    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void registerHardware(HardwareMap hwmap) {
        super.registerHardware(hwmap);
        fly = (hwmap.get(DcMotorEx.class,"fly"));
        gate = new CRServoQUS(hwmap.get(CRServo.class,"gate"));
    }

    @Override
    public void startup() {
        super.startup();

        fly.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LauncherConfig.flyPidCoef);


    }

    @Override
    public void update(LinearOpMode opmode) {
        super.update(opmode);
        switch (fly1){
            case 0:
                fly.setPower(0);
                break;
            case 1:
                fly.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,LauncherConfig.flyPidCoef);
                fly.setVelocity(LauncherConfig.TARGET_VELO);
                break;
        }
        switch(gate1){
            case 0:
                gate.queue_power(0);
                break;
            case 1:
                if(fly.getVelocity() > LauncherConfig.LAUNCHER_MIN_VELO) {
                    gate.queue_power(1);
                }
                break;
        }

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    public void shutdown(){

    }

    public void spinUp(){
        fly1 = 1;
    }

    public void spinDown(){
        fly1 = 0;
    }

    public void spinGate(){
        gate1 = 1;
    }

    public void stopGate(){
        gate1 = 0;
    }
}
