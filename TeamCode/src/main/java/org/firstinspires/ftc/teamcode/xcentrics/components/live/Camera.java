package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import static org.firstinspires.ftc.teamcode.xcentrics.components.live.CameraConfig.*;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
class CameraConfig{
    public static int PPGID = 0, PGPID = 1, GPPID = 2;
}
public class Camera extends Component {
    private HuskyLens h1;
    Pattern pattern;
    public enum Pattern{PPG,PGP,GPP}
    public Camera(Robot robot) {
        super(robot);
    }
    public void registerHardware(HardwareMap hwmap){
        super.registerHardware(hwmap);
        h1 = hwmap.get(HuskyLens.class,"h1");
    }
    public void update(OpMode opMode){
        super.update(opMode);

    }
    public void updateTelemetry(Telemetry telemetry){

    }

    private Pattern getPattern(){
        HuskyLens.Block[] blocks = h1.blocks();
        for(HuskyLens.Block block: blocks){
            if(block.id == PPGID){
                return Pattern.PPG;
            } else if(block.id == PGPID){
                return Pattern.PGP;
            } else if(block.id == GPPID){
                return Pattern.GPP;
            }
        }
        return null;
    }
}
