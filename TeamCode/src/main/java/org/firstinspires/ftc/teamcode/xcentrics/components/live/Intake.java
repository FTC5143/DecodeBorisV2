package org.firstinspires.ftc.teamcode.xcentrics.components.live;


import static org.firstinspires.ftc.teamcode.xcentrics.components.live.IntakeConfig.speed;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

@Configurable
class IntakeConfig{
    
    public static double speed = -0.5;
}


public class Intake extends Component {
    ///  MOTORS ///
    DcMotorQUS intake;

    /// CRServos ///
    private CRServoQUS in1,in2,in3,in4;
    {
        name = "intake";
    }
    //
    private int state = 0;
    
    public Intake(Robot robot) {
        super(robot);
    }
    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
        intake = new DcMotorQUS(hardwareMap.get(DcMotorEx.class,"intake"),true);
        in1 = new CRServoQUS(hardwareMap.get(CRServo.class,"transfer1"),true);
        in2 = new CRServoQUS(hardwareMap.get(CRServo.class,"transfer2"),true);
        in3 = new CRServoQUS(hardwareMap.get(CRServo.class,"transfer3"),true);
        in4 = new CRServoQUS(hardwareMap.get(CRServo.class,"intake4"),true);
        in1.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        in2.servo.setDirection(DcMotorSimple.Direction.FORWARD);
        in3.servo.setDirection(DcMotorSimple.Direction.REVERSE);
        in4.servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(LinearOpMode opMode){
        super.update(opMode);
        switch(state){
            case 0:
                setPower(0);
                break;

            case 1:
                setPower(speed);
                break;

            case 2:
                setPower(-speed);
                break;
        }
        update();
    }

    private void setPower(double power){
        in1.queue_power(power);
        in2.queue_power(power);
        in3.queue_power(power);
        in4.queue_power(power);
        intake.queue_power(power);
    }
    private void update(){
        in1.update();
        in2.update();
        in3.update();
        in4.update();
        intake.update();
    }

    public void updateTelemetry(Telemetry telemetry){
        super.updateTelemetry(telemetry);

        addData("Speed: ",speed);
    }

    public void intake(){
         state = 1;
    }
    public void stopIntake(){
         state = 0;
    }
    public void outtake(){
        state = 2;
    }
}
