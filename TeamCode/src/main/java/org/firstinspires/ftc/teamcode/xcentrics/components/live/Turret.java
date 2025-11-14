package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.control.MiniPID;
import org.firstinspires.ftc.teamcode.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.util.qus.ServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretD;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretP;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretI;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.aim;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.blueGoalTagID;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.distance;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyP;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyI;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyD;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flySpeedProportion;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.focalLength;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.hoodAngleProportion;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.redGoalTagID;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.tagHeight;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretOffset;

@Configurable
class TurretConfig{
    public static int TurretP = 0;
    public static int TurretI = 0;
    public static int TurretD = 0;
    public static int flyP = 0,flyI = 0,flyD = 0;
    public static int targetVelocity = 0;
    public static int redGoalTagID = 1,blueGoalTagID = 2;
    public static double hoodAngleProportion = 0;
    public static double flySpeedProportion = 0;
    public static double focalLength, distance, tagHeight;
    public static double turretOffset = 0;
    public static boolean aim = false;
}
public class Turret extends Component {
    private DcMotorEx fly1,fly2;
    private DcMotorQUS turret;
    private HuskyLens huskyLens;
    private ServoQUS hood;
    private MiniPID pid;

    {
        name = "turret";
    }
    public Turret(Robot robot) {
        super(robot);
    }
    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
        fly1 = hardwareMap.get(DcMotorEx.class,"fly1");
        fly2 = hardwareMap.get(DcMotorEx.class,"fly2");
        turret = new DcMotorQUS(hardwareMap.get(DcMotorEx.class,"turret"));
        huskyLens = hardwareMap.get(HuskyLens.class,"hl2");
        hood = new ServoQUS(hardwareMap.get(Servo.class,"hood"));
    }

    public void startup(){
        super.startup();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        pid = new MiniPID(TurretP, TurretI, TurretD);
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void update(OpMode opMode){
        super.update(opMode);
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for(int i = 0; i < blocks.length;i++){
            addData("Block: " + i,blocks[i].toString());
            addLine("Block center: (" + blocks[i].x + "," + blocks[i].y + ")");
        }
        //update pid coeficents
        pid.setPID(TurretP,TurretI,TurretD);
        setFlyPid(flyP,flyI,flyD);

        // only aim when we are ready
        if(aim) {
            //set target velocity
            calcFlySpeed();

            //update turret position
            calcTurretHeading();

            //update hood angle
            calcHoodAngle();
        }
        //stop turret from moving to prevent damage to the robot when not updating pid
        else {
            turret.queue_power(0);
        }

        //update qus
        update();
    }

    private void setFlyPid(double p,double i,double d){
        fly1.setVelocityPIDFCoefficients(p,i,d,0);
        fly1.setVelocityPIDFCoefficients(p,i,d,0);
    }
    private void setVelocity(double velocity){
        fly1.setVelocity(velocity);
        fly2.setVelocity(velocity);
    }

    //get the current tag
    private HuskyLens.Block getBlock(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for(HuskyLens.Block block: blocks){
            if(robot.isRed() && block.id == redGoalTagID){
                return block;
            } else if(!robot.isRed() && block.id == blueGoalTagID){
                return block;
            }
        }
        return null;
    }
    //calc distance from tag
    public void getDistance(){
        if(getBlock() != null) {
            distance = (tagHeight * focalLength) / getBlock().height;
        }
    }
    //calc hood angle
    private void calcHoodAngle(){
        hood.queue_position(distance * hoodAngleProportion);
    }
    //calculate turret heading
    private void calcTurretHeading(){
        if(getBlock() != null) {
            turret.queue_power(pid.getOutput(getBlock().x,turretOffset));
        }
    }
    //calculate the fly speed
    private void calcFlySpeed(){
        setVelocity(distance * flySpeedProportion);
    }
    //update qus
    private void update(){
        turret.update();
        hood.update();
    }

}
