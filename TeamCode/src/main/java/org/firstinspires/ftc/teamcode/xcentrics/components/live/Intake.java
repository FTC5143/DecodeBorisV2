package org.firstinspires.ftc.teamcode.xcentrics.components.live;


import static org.firstinspires.ftc.teamcode.xcentrics.components.live.IntakeConfig.speed;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
@Configurable
class IntakeConfig{
    public static double speed;
}

public class Intake extends Component {
    ///  MOTORS ///
    DcMotorQUS intake;

    {
        name = "intake";
    }
    public Intake(Robot robot) {
        super(robot);
    }
    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
        intake = new DcMotorQUS(hardwareMap.get(DcMotorEx.class,"intake"));
    }

    public void update(OpMode opMode){
        super.update(opMode);
        intake.queue_power(speed);
    }

    public void setPower(double power){
        speed = power;
    }

}
