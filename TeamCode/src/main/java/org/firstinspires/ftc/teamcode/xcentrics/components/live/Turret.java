package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.*;

@Configurable
class TurretConfig {

    // ------------------------------
    // PID Configuration
    // ------------------------------
    //public static double TurretP = 0, TurretI = 0, TurretD = 0;
  public static double flyP = 13, flyI = 0, flyD = 0;


    // ------------------------------
    // Shooter State
    // ------------------------------
    public static boolean shooterReady = false;
    public static boolean velocityReady = false;
    public static boolean angleReady = false;

    // ------------------------------
    // Turret Encoder Configuration
    // ------------------------------

    public static double ticksPerTurretRotation = 6610;      // encoder ticks per 360°
    public static double turretHeading;               // current turret heading (deg)
    public static double turretTarget;                // target turret heading (ticks)

    // ------------------------------
    // Other Servo Positions
    // ------------------------------
    public static double kickPos = 0, safetyOff = 0, safetyOn = 1;
    //public static boolean aim = false;               // is turret tracking target
    public static boolean canSpin = true;           // flywheel allowed to spin
    public static Pose testPose = new Pose(9,9,Math.toRadians(0));
    public static  double power = 0;
    public static PIDFCoefficients turretPIDCoef = new PIDFCoefficients(0.01,0,0,0),
    flyPIDFCoef = new PIDFCoefficients(0,0,0,0);
    public static double targetVelocity = 999999999;;
}
@Configurable

public class Turret extends Component {
    public static double a = 0.5 ,b =0 ,c = 0.0;

    // ------------------------------
    // Hardware
    // ------------------------------
    private DcMotorEx fly1,fly2;
    private DcMotorQUS turret;
    private HuskyLens husky;
    public double turretOffset = 0;            // calibration offset
    private ServoQUS hood1, hood2, kicker, safety;

   // private MiniPID flyPID;   // PID controllers
    private PIDFController turretPID,flyPID;

    private LiveRobot robot;             // reference to robot

    private Pose redGoal = new Pose(131,137), blueGoal = new Pose(12,136);      // goal positions
    public static double distance;             // distance to goal

    private double lastHoodCmd = 0;     // previous hood angle
    private double prevFilteredTarget = 0; // for low-pass filtering
    public  boolean autoAim = true;
    public static boolean resetEncoder = false;
    public static boolean Debug = false;
    public static double servoPos = 0;

    {
        name = "turret";
    }

    public Turret(Robot robot, LiveRobot live) {
        super(robot);
        this.robot = live;
    }

    // ------------------------------
    // Hardware Registration
    // ------------------------------
    @Override
    public void registerHardware(HardwareMap map) {
        super.registerHardware(map);

        fly1 = map.get(DcMotorEx.class, "fly1");
        fly2 = map.get(DcMotorEx.class,"fly2");
        turret = new DcMotorQUS(map.get(DcMotorEx.class, "turret"));
        turret.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        husky = map.get(HuskyLens.class, "hl1");

        hood1 = new ServoQUS(map.get(Servo.class, "H1"));
        hood2 = new ServoQUS(map.get(Servo.class, "H2"));
        kicker = new ServoQUS(map.get(Servo.class, "kicker"));
        safety = new ServoQUS(map.get(Servo.class, "safety"));
    }

    // ------------------------------
    // Initialization
    // ------------------------------
    @Override
    public void startup() {
        super.startup();

        turretPID = new PIDFController(turretPIDCoef);
        flyPID = new PIDFController(flyPIDFCoef);

        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        kicker.queue_position(1);

        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.motor.setPower(0);
        updateAll();
    }

    // ------------------------------
    // Main Update Loop
    // ------------------------------
    @Override
    public void update(OpMode opMode) {
        super.update(opMode);

        turretPID.setCoefficients(turretPIDCoef);
        fly1.setVelocityPIDFCoefficients(flyP,flyI,flyD,0);
        fly2.setVelocityPIDFCoefficients(flyP,flyI,flyD,0);

        updateTurretHeading();  // update turret heading relative to robot


            if(resetEncoder){
                resetEncoder();
                resetEncoder = false;
            }


            computeDistance();

            if(autoAim) {
                computeTurretTarget();
                computeFlySpeed();
            }
            if(Debug){
                robot.follower.setPose(testPose);
            }
            computeHoodAngle();

            turretPID.setTargetPosition(turretTarget);
            turretPID.updatePosition(turret.motor.getCurrentPosition());
        power = turretPID.run();
        if (autoAim) {
            turret.queue_power(turretPID.run());
        } else {
            turret.queue_power(0);
            fly1.setVelocity(0);
            fly2.setVelocity(0);
        }
        computeReadyState();
        updateAll();
    }

    // ------------------------------
    // Telemetry
    // ------------------------------
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        addLine("Shooter Ready: " + shooterReady);
        addLine("Velocity Ready: " + velocityReady);
        addLine("Angle Ready: " + angleReady);

        addData("Turret Heading", turretHeading);
        addData("TargetTicks", turretTarget);
        addData("Distance", distance);
        addData("Fly1Vel", fly1.getVelocity());
        addData("Fly2Vel",fly2.getVelocity());
        addData("FlyTargetVel",targetVelocity);

        addData("TurretTicks",turret.motor.getCurrentPosition());
    }

    // ------------------------------
    // Turret Heading
    // ------------------------------
    private void updateTurretHeading() {
        // Convert encoder ticks to relative degrees
        double ticks = turret.motor.getCurrentPosition();
        double turretRelDeg = ticks * (360.0 / ticksPerTurretRotation);

        // Add robot heading
        double robotHeadingDeg = Math.toDegrees(robot.follower.getPose().getHeading());
        turretHeading = AngleUnit.normalizeDegrees(
                robotHeadingDeg + (-turretRelDeg) + turretOffset
        );
    }

    // Compute turret target with motion compensation
    private void computeTurretTarget() {
        Pose p = robot.follower.getPose();
        double targetRads = robot.isRed() ?
                Math.atan2(redGoal.getY() - p.getY(), redGoal.getX() - p.getX()) :
                Math.atan2(blueGoal.getY() - p.getY(), blueGoal.getX() - p.getX());

        double targetDegField = Math.toDegrees(targetRads);
        double robotHeadingDeg = Math.toDegrees(p.getHeading());

        double targetDegTurret = AngleUnit.normalizeDegrees(
                targetDegField - robotHeadingDeg - turretOffset
        );

        double rawTicks = targetDegTurret * (ticksPerTurretRotation / 360.0);
        double minTicks = -4633;
        double maxTicks = 3917;

        turretTarget = Math.max(minTicks, Math.min(maxTicks, rawTicks));
    }

    // ------------------------------
    // Hood / Flywheel Physics
    // ------------------------------
    private void computeHoodAngle() {
        servoPos = a + (b * distance) + (c * c * distance);
        hood1.queue_position(servoPos);
        hood2.queue_position(-servoPos);
    }
    public void manualTurret(){
        autoAim = false;
        turretTarget = 0;
    }
    private void computeFlySpeed() {
        fly1.setVelocity(targetVelocity);
        fly2.setVelocity(targetVelocity);
    }
    public void resetEncoder(){
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // ------------------------------
    // Ready State
    // ------------------------------
    private void computeReadyState() {
    }

    // ------------------------------
    // Launch / Kicker
    // ------------------------------
    public void launch() {
        kicker.queue_position(0);
        updateAll();
        halt(0.5);
        kicker.queue_position(1);
        updateAll();
    }


    // ------------------------------
    // Utilities
    // ------------------------------
    private void computeDistance() {
        Pose p = robot.follower.getPose();
        distance = robot.isRed() ? redGoal.distanceFrom(p) : blueGoal.distanceFrom(p);
    }

    private void updateAll() {
        turret.update();
        hood1.update();
        hood2.update();
        kicker.update();
        safety.update();
    }
    @Override
    public void shutdown() {
        super.shutdown();
        safety.queue_position(safetyOn);
    }
    public Runnable aim(){
        return new Runnable() {
            @Override
            public void run() {
                autoAim = true;
            }
        };
    }
    public Runnable stopAim(){
        return new Runnable() {
            @Override
            public void run() {
                autoAim = false;
            }
        };
    }
}
