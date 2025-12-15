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
    public static double TurretP = 0, TurretI = 0, TurretD = 0;
    public static double flyP = 0, flyI = 0, flyD = 0;

    // ------------------------------
    // Shooter Physics / Hood
    // ------------------------------
    public static double shooterHeight = 0.30;    // meters
    public static double goalHeight = 1.22;       // meters
    public static double hoodMinAngle = 10;       // degrees
    public static double hoodMaxAngle = 45;       // degrees
    public static double hoodOffset = 0;          // calibration offset

   // public static double wheelRadius = 0.045;     // meters
    public static double rpmToVelocity = 0.0012;  // conversion factor

    // ------------------------------
    // Ready Thresholds
    // ------------------------------
    public static double readyVelocityTolerance = 35; // flywheel RPM tolerance
    public static double readyAngleTolerance = 1.5;   // hood angle tolerance

    // ------------------------------
    // Shooter State
    // ------------------------------
    public static boolean shooterReady = false;
    public static boolean velocityReady = false;
    public static boolean angleReady = false;

    // ------------------------------
    // Anti-Hit System
    // ------------------------------
    public static boolean hitDetected = false;        // is turret frozen due to hit
   /* public static double hitFreezeTime = 0.35;        // time to stop turret (s)
    public static double hitCooldown = 0.20;          // slow ramp time after hit
    public static long lastHitTime = 0;

    public static double jerkThreshold = 0.25;        // rad/s² for heading jerk
    public static double transJerkThreshold = 0.40;   // m/s² for translation jerk
    public static double backlashThreshold = 15;      // ticks/sec spike threshold
*/
    // ------------------------------
    // Turret Encoder Configuration
    // ------------------------------
    public static double turretOffset = 0;            // calibration offset
    public static double ticksPerTurretRotation = 6610;      // encoder ticks per 360°
    public static double turretHeading;               // current turret heading (deg)
    public static double turretTarget;                // target turret heading (ticks)
    public static double minRotation = 15;             // min turret angle 15
    public static double maxRotation = 360 - 15;           // max turret angle

    // ------------------------------
    // Other Servo Positions
    // ------------------------------
    public static double kickPos = 0, safetyOff = 0, safetyOn = 1;
    public static boolean aim = false;               // is turret tracking target
    public static boolean canSpin = true;           // flywheel allowed to spin
    //public static double turretTolerance = 0, flyTolerance = 0; // error tolerances
    public static double turretError, flyError;       // current errors
    public static Pose testPose = new Pose(72,23,Math.toRadians(0));
    public static  double power = 0;
    public static PIDFCoefficients turretPIDCoef = new PIDFCoefficients(0.001,0,0,0);
}
@Configurable

public class Turret extends Component {

    // ------------------------------
    // Hardware
    // ------------------------------
    private DcMotorEx fly;
    private DcMotorQUS turret;
    private HuskyLens husky;

    private ServoQUS hood1, hood2, kicker, safety;

    private MiniPID flyPID;   // PID controllers
    private PIDFController turretPID;

    private LiveRobot robot;             // reference to robot

    private Pose redGoal = new Pose(131,137), blueGoal = new Pose(12,136);      // goal positions
    public static double distance;             // distance to goal

    private double lastHoodCmd = 0;     // previous hood angle
    private double prevFilteredTarget = 0; // for low-pass filtering
    public  boolean autoAim = true;

    // ------------------------------
    // Kinematics History (Pedro Pathing)
    // ------------------------------
    private double prevX = 0, prevY = 0, prevHeading = 0;
    private double prevVx = 0, prevVy = 0, prevHeadingVel = 0;
    private long prevTime = 0;

    // computed kinematics
    private double k_vx, k_vy;
    private double k_ax, k_ay;
    private double k_headingVel, k_headingAcc;
    private double flyPower = 0;
    public static boolean Debug = false;

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

        fly = map.get(DcMotorEx.class, "fly");
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
        flyPID = new MiniPID(flyP, flyI, flyD);

        fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.motor.setPower(0);
    }

    // ------------------------------
    // Main Update Loop
    // ------------------------------
    @Override
    public void update(OpMode opMode) {
        super.update(opMode);

        turretPID.setCoefficients(turretPIDCoef);
        flyPID.setPID(flyP, flyI, flyD);

        updateTurretHeading();  // update turret heading relative to robot
        updateKinematics();     // compute internal velocity/acceleration
        //detectHit();            // freeze turret if hit detected




            computeDistance();

            if(autoAim) {
                computeTurretTarget();
            }
            if(Debug){
                robot.follower.setPose(testPose);
            }
            computeHoodAngle();
            computeFlySpeed();
            turretPID.setTargetPosition(turretTarget);
            turretPID.updatePosition(turret.motor.getCurrentPosition());
        power = turretPID.run();
        turret.queue_power(turretPID.run());

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
        addLine("HitDetected: " + hitDetected);

        addData("Turret Heading", turretHeading);
        addData("TargetTicks", turretTarget);
        addData("Distance", distance);
        addData("FlyVel", fly.getVelocity());

        addData("TurretTicks",turret.motor.getCurrentPosition());
        addData("TurretPower",power);
        addData("ActualPower",turret.motor.getPower());
        addData("FlyPower",fly.getPower());
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
        // Simple ballistic formula for hood angle
        double D = distance;
        double h0 = shooterHeight;
        double h1 = goalHeight;

        double v = fly.getVelocity() * rpmToVelocity;
        if (v < 1) v = 1; // prevent division by zero

        double g = 9.81;
        double term = (g * D * D) / (2 * v * v);
        double linear = (h1 - h0) / D;

        double hoodRad = Math.atan(term + linear);
        double hoodDeg = Math.toDegrees(hoodRad) + hoodOffset;
        hoodDeg = clamp(hoodDeg, hoodMinAngle, hoodMaxAngle);

        double servoPos = hoodDeg / 180.0;
        hood1.queue_position(servoPos);
        hood2.queue_position(servoPos);

        double hoodErr = Math.abs(hoodDeg - lastHoodCmd);
        angleReady = hoodErr < readyAngleTolerance;

        lastHoodCmd = hoodDeg;
    }
    public void manualTurret(){
        autoAim = false;
        turretTarget = 0;
    }
    private void computeFlySpeed() {
        // Distance-based flywheel speed
        double D = distance;
        double k1 = 6.0, k2 = 12.0;
        double targetVel = k1 * D + k2;

        double targetRPM = targetVel / rpmToVelocity;
        double currentRPM = fly.getVelocity();

        double diff = targetRPM - currentRPM;

        double pid = flyPID.getOutput(currentRPM, targetRPM);

        fly.setPower(pid);

        flyError = Math.abs(currentRPM - targetRPM);
        velocityReady = flyError < readyVelocityTolerance;
    }

    // ------------------------------
    // Anti-Hit Detection
    // ------------------------------
    private void updateKinematics() {
        long now = System.currentTimeMillis();
        if (prevTime == 0) {
            Pose p = robot.follower.getPose();
            prevX = p.getX(); prevY = p.getY(); prevHeading = p.getHeading();
            prevTime = now;
            return;
        }

        double dt = (now - prevTime) / 1000.0;
        if (dt <= 0) dt = 0.001;

        Pose p = robot.follower.getPose();
        double dx = p.getX() - prevX;
        double dy = p.getY() - prevY;
        double dheading = AngleUnit.normalizeRadians(p.getHeading() - prevHeading);

        double vx = dx / dt;
        double vy = dy / dt;
        double headingVel = dheading / dt;

        double ax = (vx - prevVx) / dt;
        double ay = (vy - prevVy) / dt;
        double headingAcc = (headingVel - prevHeadingVel) / dt;

        k_vx = vx; k_vy = vy; k_ax = ax; k_ay = ay;
        k_headingVel = headingVel; k_headingAcc = headingAcc;

        prevX = p.getX(); prevY = p.getY(); prevHeading = p.getHeading();
        prevVx = vx; prevVy = vy; prevHeadingVel = headingVel;
        prevTime = now;
    }

//    private void detectHit() {
//        long now = System.currentTimeMillis();
//
//        boolean imuHit = Math.abs(k_headingAcc) > jerkThreshold;
//        boolean transHit = Math.abs(k_ax) > transJerkThreshold || Math.abs(k_ay) > transJerkThreshold;
//        double tickVel = turret.motor.getVelocity();
//        boolean backlash = Math.abs(tickVel) > backlashThreshold;
//
//        if (imuHit || transHit || backlash) {
//            hitDetected = true;
//            lastHitTime = now;
//        }
//
//        if (hitDetected && now - lastHitTime > (hitFreezeTime + hitCooldown) * 1000.0) {
//            hitDetected = false;
//        }
//    }

    // ------------------------------
    // Ready State
    // ------------------------------
    private void computeReadyState() {
        shooterReady = velocityReady && angleReady && !hitDetected;
    }

    // ------------------------------
    // Launch / Kicker
    // ------------------------------
    public void launch() {
        if (!shooterReady) return;
        if (safety.servo.getPosition() == safetyOff) {
            kicker.queue_position(kickPos);
            canSpin = false;
        }
    }

    public void resetKicker() {
        if (!canSpin) {
            kicker.queue_position(0);
            canSpin = true;
        }
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

    private double lowPass(double input, double alpha) {
        return prevFilteredTarget = alpha * input + (1 - alpha) * prevFilteredTarget;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void shutdown() {
        super.shutdown();
        fly.setVelocity(0);
        safety.queue_position(safetyOn);
    }
}
