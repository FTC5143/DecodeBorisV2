package org.firstinspires.ftc.teamcode.xcentrics.components.live;


import static org.firstinspires.ftc.teamcode.xcentrics.components.live.IntakeConfig.speed;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
@Configurable
class IntakeConfig{
    public static double speed;
}

public class Intake extends Component {
    ///  MOTORS ///
    DcMotorQUS intake;

    /// CRServos ///
    private CRServoQUS in1,in2;
    {
        name = "intake";
    }
    public Intake(Robot robot) {
        super(robot);
    }
    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
        intake = new DcMotorQUS(hardwareMap.get(DcMotorEx.class,"intake"));
        in1 = new CRServoQUS(hardwareMap.get(CRServo.class,"Lin"));
        in2 = new CRServoQUS(hardwareMap.get(CRServo.class,"Rin"));
    }

    public void update(OpMode opMode){
        super.update(opMode);

        update();
    }

    public void setPower(double power){
        in1.queue_power(power);
        in2.queue_power(power);
        intake.queue_power(power);
    }
    private void update(){
        in1.update();
        in2.update();
        intake.update();
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryManager telemetryManager){
        super.updateTelemetry(telemetry,telemetryManager);

        addData("Speed: ",speed);
    }
    public void intake(Spindexer spindexer){
        spindexer.intake();
        setPower(speed);
    }
    public void outtake(){
        setPower(-speed);
    }

}
