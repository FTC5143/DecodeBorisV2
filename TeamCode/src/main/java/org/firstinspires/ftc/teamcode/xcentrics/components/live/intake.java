package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.ServoQUS;

public class intake extends Component {
    private DcMotorQUS intake;
    private CRServoQUS inl, inr;


    public intake(Robot robot) {
        super(robot);
    }

    @Override
    public void registerHardware(HardwareMap hwmap) {
        super.registerHardware(hwmap);
        intake = new DcMotorQUS(hwmap.get(DcMotorEx.class,"intake"));
        inl = new CRServoQUS(hwmap.get(CRServo.class,"inl"));
        inr = new CRServoQUS(hwmap.get(CRServo.class,"inr"));

    }

    @Override
    public void startup() {
        super.startup();
    }

    @Override
    public void update(LinearOpMode opmode) {
        super.update(opmode);
        intake.update();
        inl.update();
        inr.update();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    private void setPower(double power){
        intake.queue_power(power);
        inl.queue_power(-power);
        inr.queue_power(power);
    }

    public void intake(){
        setPower(1);
    }

    public void stopIntake(){
        setPower(0);
    }
    public void reverse(){
        setPower(-1);
    }
}
