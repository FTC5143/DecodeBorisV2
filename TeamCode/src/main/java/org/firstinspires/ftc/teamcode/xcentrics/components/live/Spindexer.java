//package org.firstinspires.ftc.teamcode.xcentrics.components.live;
//
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.D;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.I;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.artifacts;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.ballTicks;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.blueID;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.currentPosition;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.redID;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.spinPIDcoef;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.canSpin;
//
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
//import org.firstinspires.ftc.teamcode.xcentrics.util.control.MiniPID;
//import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;
//import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpindexerConfig.P;
//@Configurable
//class SpindexerConfig {
//    public static int[] ballTicks = {0, 1000, 2000};
//    public static double P = 0, I = 0, D = 0;
//    public static Artifact[] artifacts = new Artifact[3];
//    public static double currentPosition = 0;
//    public static double blueID = 0, redID = 1;
//    public static PIDFCoefficients spinPIDcoef = new PIDFCoefficients(0,0,0,0);
//}
//
//enum Artifact{GREEN, PURPLE, EMPTY}
//enum spinPos{ONE,TWO,TREE,SPINING}
//
//public class Spindexer extends Component {
//    /// CRServos ///
//    private CRServoQUS spin;
//    /// Encoders ///
//    private DcMotorEx encoder;
//    /// Camera ///
//    private HuskyLens huskyLens;
//
//    private PIDFController pid;
//    {
//        name = "Spindexer";
//    }
//    public Spindexer(Robot robot) {
//        super(robot);
//    }
//
//    public void registerHardware(HardwareMap hardwareMap){
//        super.registerHardware(hardwareMap);
//        spin = new CRServoQUS(hardwareMap.get(CRServo.class,"Spin"));
//        huskyLens = hardwareMap.get(HuskyLens.class,"hl2");
//        encoder = hardwareMap.get(DcMotorEx.class,"FL");
//    }
//
//    public void startup(){
//        super.startup();
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
//        pid.setCoefficients(spinPIDcoef);
//    }
//
//    public void update(OpMode opMode){
//        super.update(opMode);
//
//        //update current position
//        currentPosition = encoder.getCurrentPosition();
//        //update
//        if(!canSpin){
//            spin.queue_power(0);
//            update();
//        } else {
//            update();
//        }
//    }
//
//    public void updateTelemetry(Telemetry telemetry){
//        super.updateTelemetry(telemetry);
//
//        addData("Can spin: ", canSpin);
//        addData("Current position: ",currentPosition);
//        addData("Current artifact: ",getCurrentBall());
//        addData("Artifact 1: ",artifacts[0]);
//        addData("Artifact 2: ",artifacts[1]);
//        addData("Artifact 3: ",artifacts[3]);
//
//
//    }
//
//    public void shutdown(){
//        super.shutdown();
//        canSpin = false;
//    }
//    private HuskyLens.Block getBlock(){
//        HuskyLens.Block[] blocks = huskyLens.blocks();
//        for(HuskyLens.Block block: blocks){
//            if(robot.isRed() && block.id == redID){
//                return block;
//            } else if(!robot.isRed() && block.id == blueID){
//                return block;
//            }
//        }
//        return null;
//    }
//    private Artifact getCurrentBall(){
//        if(currentPosition == ballTicks[0]){
//            return artifacts[0];
//        } else if(currentPosition == ballTicks[1]){
//            return artifacts[1];
//        } else {
//            return artifacts[2];
//        }
//
//    }
//    private void setCurrentBall(Artifact artifact){
//        double currentPose = encoder.getCurrentPosition();
//        if(currentPose % ballTicks[0] >= 2 || currentPose % ballTicks[0] <=2){
//            artifacts[0] = artifact;
//        } else if(currentPose == ballTicks[1]){
//            artifacts[1] = artifact;
//        } else if(currentPose == ballTicks[2]){
//            artifacts[2] = artifact;
//        }
//    }
//    private void clasify(){
//        halt(0.5);
//        if(getBlock() == null){
//            setCurrentBall(Artifact.EMPTY);
//        } else if(getBlock().id == 1){
//            setCurrentBall(Artifact.PURPLE);
//        } else if(getBlock().id == 2){
//            setCurrentBall(Artifact.GREEN);
//        }
//    }
//    public void goToBall(int id){
//        if(id <= 0) {
//            pid.updateError(encoder.getCurrentPosition() % ballTicks[id] - encoder.getCurrentPosition());
//            spin.queue_power(pid.run());
//        }
//    }
//    private void update(){
//        spin.update();
//    }
//
//    public void index(){
//        goToBall(0);
//        if(currentPosition == ballTicks[1]){
//            halt(0.01);
//            clasify();
//            goToBall(1);
//            if(currentPosition == ballTicks[2]){
//                halt(0.01);
//                clasify();
//                goToBall(2);
//                if(currentPosition == ballTicks[2]){
//                    halt(0.01);
//                    clasify();
//                }
//            }
//        }
//    }
//    public void intake(){
//        spin.queue_power(0.5);
//    }
//
//
//}
