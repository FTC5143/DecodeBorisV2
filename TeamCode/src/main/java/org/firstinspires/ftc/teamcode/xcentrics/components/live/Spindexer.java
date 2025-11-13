package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.qus.CRServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

public class Spindexer extends Component {
    /// CRServos ///
    private CRServoQUS spin;
    /// Encoders ///
    private DcMotor encoder;
    public Spindexer(Robot robot) {
        super(robot);
    }

    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
    }


}
