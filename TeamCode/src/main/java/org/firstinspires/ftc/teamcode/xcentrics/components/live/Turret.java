package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.util.control.MiniPID;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.DcMotorQUS;
import org.firstinspires.ftc.teamcode.xcentrics.util.qus.ServoQUS;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretD;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretP;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.TurretI;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.aim;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.blueGoalTagID;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.canSpin;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyError;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyP;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyI;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyD;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.flyTolerance;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.kickPos;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.maxRotation;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.minRotation;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.redGoalTagID;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.safetyOff;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.safetyOn;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.servoRange;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.targetVelocity;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.ticksPerTurretRotation;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretError;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretHeading;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretOffset;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretTarget;
import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.turretTolerance;

@Configurable
class TurretConfig{
    public static double TurretP = 0;
    public static double TurretI = 0;
    public static double TurretD = 0;
    public static double flyP = 0,flyI = 0,flyD = 0;
    public static int targetVelocity = 0;
    public static int redGoalTagID = 1,blueGoalTagID = 2;
    public static int servoRange = 360;
    public static double turretOffset = 0;
    public static double kickPos = 0,safetyOff = 0, safetyOn = 1;
    public static boolean aim = false;
    public static double turretTolerance = 0,flyTolerance = 0;
    public static double turretError, flyError;
    public static boolean canSpin = true;
    public static double turretHeading,ticksPerTurretRotation,turretTarget;
    public static double maxRotation = 270, minRotation = 0;
}
public class Turret extends Component {
    private DcMotorEx fly;
    private DcMotorQUS turret;
    private HuskyLens huskyLens;
    private ServoQUS hood1,hood2, kicker,safety;
    private MiniPID pid;
    private LiveRobot robot;
    private Pose redGoal, blueGoal;
    private double distance;



    {
        name = "turret";
    }
    public Turret(Robot robot,LiveRobot roobot) {
        super(robot);
        this.robot = roobot;
    }
    public void registerHardware(HardwareMap hardwareMap){
        super.registerHardware(hardwareMap);
        fly =           hardwareMap.get(DcMotorEx.class,"fly");
        turret =        new DcMotorQUS(hardwareMap.get(DcMotorEx.class,"turret"));
        huskyLens =     hardwareMap.get(HuskyLens.class,"hl1");
        hood1 =         new ServoQUS(hardwareMap.get(Servo.class,"H1"));
        hood2 =         new ServoQUS(hardwareMap.get(Servo.class,"H2"));
        kicker =        new ServoQUS(hardwareMap.get(Servo.class,"kicker"));
        safety =        new ServoQUS(hardwareMap.get(Servo.class,"safety"));
    }

    public void startup(){
        super.startup();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        pid = new MiniPID(TurretP, TurretI, TurretD);
        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        //update current turret heading
        setHeading(turret.motor.getCurrentPosition());
        // only aim when we are ready
        if(aim) {
            //update distance
            getDistance();

            //update turret position
            calcTurretHeading();

            //update hood angle
            calcHoodAngle();

            //set target velocity
            calcFlySpeed();
        }
        //stop turret from moving to prevent damage to the robot when not updating pid
        else {
            turret.queue_power(0);
        }

        //update qus
        update();
    }

    public void updateTelemetry(Telemetry telemetry){
        super.updateTelemetry(telemetry);

        addLine("Fly PID: P: " + flyP + ",I: " + flyI + ",D: " + flyD);
        addData("Fly Velocity: ", fly.getVelocity());
        addLine("Turret PID: P: " + TurretP + ",I: " + TurretI + ",D: " + TurretD);

    }

    //shutdown the turret
    public void shutdown(){
        super.shutdown();
        setVelocity(0);
        engageSafety();
    }

    private void setFlyPid(double p,double i,double d){
        fly.setVelocityPIDFCoefficients(p,i,d,0);
        fly.setVelocityPIDFCoefficients(p,i,d,0);
    }
    private void setVelocity(double velocity){
        fly.setVelocity(velocity, AngleUnit.RADIANS);
    }

    //calc distance from tag
    public void getDistance(){
        if(robot.isRed()) {
            distance = redGoal.distanceFrom(robot.follower.getPose());
        } else {
            distance = blueGoal.distanceFrom(robot.follower.getPose());
        }
    }
    //calc hood angle
    private void calcHoodAngle(){
        //implament formula here
    }
    //calculate turret heading
    private void calcTurretHeading() {

        Pose robotPose = robot.follower.getPose();
        double targetRadians;

        if (robot.isRed()) {
            targetRadians = Math.atan2(
                    redGoal.getY() - robotPose.getY(),
                    redGoal.getX() - robotPose.getX());
        } else {
            targetRadians = Math.atan2(
                    blueGoal.getY() - robotPose.getY(),
                    blueGoal.getX() - robotPose.getX());
        }

        setTarget(targetRadians);

        // PID output is power
        double output = pid.getOutput(turretHeading, turretTarget);

        if (turretTarget >= minRotation && turretTarget <= maxRotation) {
            turret.queue_power(output);
        } else {
            turret.queue_power(0);  // don't damage turret
        }
    }
    //calculate the fly speed
    private void calcFlySpeed() {
        //implaent formula here
    }
    //update qus
    private void update(){
        turret.update();
        hood1.update();
        hood2.update();
        kicker.update();
        safety.update();
    }
    //launch an artifact
    public void launch(){
        if(turretError < turretTolerance && flyError < flyTolerance){
            if(safety.servo.getPosition() == safetyOff){
                canSpin = false;
                kicker.queue_position(kickPos);
            }
        }
    }
    //reset kicker
    public void resetKicker(){
        kicker.queue_position(0);
        canSpin = true;
    }
    //allow for the turret to track the goal
    public void aim(){
        aim = true;
    }
    public void stopAim(){
        aim = false;
    }
    //disengage the safety
    public void disengageSafety(){
        safety.queue_position(safetyOff);
    }
    //engage the safety
    public void engageSafety(){
        safety.queue_position(safetyOn);
    }

    private void setHeading(double encoderTicks) {
        // turret angle relative to the robot (deg)
        double turretRelativeDeg = encoderTicks * (360.0 / ticksPerTurretRotation);

        // robot field heading (Pedro heading is radians CCW+)
        double robotHeadingDeg = Math.toDegrees(robot.follower.getPose().getHeading());

        // combine, apply optional mechanical offset
        turretHeading =
                AngleUnit.normalizeDegrees(robotHeadingDeg + turretRelativeDeg + turretOffset);
    }

    private void setTarget(double targetRadians) {

        // angle (deg) from robot to goal (field relative)
        double targetDeg = Math.toDegrees(targetRadians);

        // convert to angle relative to turret’s 0° on the robot
        targetDeg -= Math.toDegrees(robot.follower.getPose().getHeading());
        targetDeg -= turretOffset;

        // convert degrees → encoder ticks
        turretTarget = targetDeg * (ticksPerTurretRotation / 360.0);
    }

}
