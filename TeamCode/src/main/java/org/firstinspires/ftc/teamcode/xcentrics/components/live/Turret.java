package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

import static org.firstinspires.ftc.teamcode.xcentrics.components.live.TurretConfig.*;

@Configurable
class TurretConfig {
    public static double TurretP = 0, TurretI = 0, TurretD = 0;
    public static double flyP = 0, flyI = 0, flyD = 0;

    public static int targetVelocity = 0;

    public static int redGoalTagID = 1, blueGoalTagID = 2;
    public static int servoRange = 360;

    public static double turretOffset = 0;   // mech offset in DEGREES
    public static double kickPos = 0, safetyOff = 0, safetyOn = 1;

    public static boolean aim = false;

    public static double turretTolerance = 0, flyTolerance = 0;
    public static double turretError, flyError;

    public static boolean canSpin = true;

    public static double turretHeading;   // FIELD RELATIVE degrees
    public static double ticksPerTurretRotation;
    public static double turretTarget;    // ENCODER TICKS target

    public static double maxRotation = 270, minRotation = 0;
}

public class Turret extends Component {

    private DcMotorEx fly;
    private DcMotorQUS turret;

    private HuskyLens huskyLens;
    private ServoQUS hood1, hood2, kicker, safety;

    private MiniPID pid;
    private LiveRobot robot;

    private Pose redGoal, blueGoal;
    private double distance;

    {
        name = "turret";
    }

    public Turret(Robot robot, LiveRobot roobot) {
        super(robot);
        this.robot = roobot;
    }

    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        super.registerHardware(hardwareMap);

        fly = hardwareMap.get(DcMotorEx.class, "fly");
        turret = new DcMotorQUS(hardwareMap.get(DcMotorEx.class, "turret"));

        huskyLens = hardwareMap.get(HuskyLens.class, "hl1");

        hood1 = new ServoQUS(hardwareMap.get(Servo.class, "H1"));
        hood2 = new ServoQUS(hardwareMap.get(Servo.class, "H2"));
        kicker = new ServoQUS(hardwareMap.get(Servo.class, "kicker"));
        safety = new ServoQUS(hardwareMap.get(Servo.class, "safety"));
    }

    @Override
    public void startup() {
        super.startup();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        pid = new MiniPID(TurretP, TurretI, TurretD);
        fly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update(OpMode opMode) {
        super.update(opMode);

        pid.setPID(TurretP, TurretI, TurretD);
        setFlyPID(flyP, flyI, flyD);

        // Update turretHeading from encoder + robot heading
        updateTurretHeading();

        if (aim) {
            computeDistance();
            computeTurretTarget();
            computeHoodAngle();
            computeFlySpeed();
        } else {
            turret.queue_power(0);
        }

        updateQUS();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        addLine("Turret PID: P=" + TurretP + " I=" + TurretI + " D=" + TurretD);
        addLine("Fly PID: P=" + flyP + " I=" + flyI + " D=" + flyD);
        addData("Fly Velocity", fly.getVelocity());
        addData("Turret Heading (°)", turretHeading);
        addData("Turret Target (ticks)", turretTarget);
    }

    // ------------------------
    // CORE TURRET MATH
    // ------------------------

    private void updateTurretHeading() {
        double ticks = turret.motor.getCurrentPosition();

        // CLOCKWISE is POSITIVE → encoder ticks directly map to +degrees
        double turretRelativeDeg = ticks * (360.0 / ticksPerTurretRotation);

        // Pedro heading is in radians CCW+, convert to degrees
        double robotHeadingDeg = Math.toDegrees(robot.follower.getPose().getHeading());

        // FIELD RELATIVE TURRET HEADING
        turretHeading = AngleUnit.normalizeDegrees(
                robotHeadingDeg + (-turretRelativeDeg) + turretOffset
        );
        // (-turretRelativeDeg) because robot CCW+ but turret CW+
    }

    private void computeTurretTarget() {
        Pose pose = robot.follower.getPose();

        double targetRadians;

        if (robot.isRed()) {
            targetRadians = Math.atan2(
                    redGoal.getY() - pose.getY(),
                    redGoal.getX() - pose.getX());
        } else {
            targetRadians = Math.atan2(
                    blueGoal.getY() - pose.getY(),
                    blueGoal.getX() - pose.getX());
        }

        double targetDegField = Math.toDegrees(targetRadians);  // field-relative angle to goal
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // Convert field-relative → robot-relative → turret-relative
        double targetDegTurret = AngleUnit.normalizeDegrees(
                targetDegField - robotHeadingDeg - turretOffset
        );

        // Convert turretDegrees → ticks
        turretTarget = targetDegTurret * (ticksPerTurretRotation / 360.0);

        // Apply rotation limits (in ticks)
        double minTicks = minRotation * (ticksPerTurretRotation / 360.0);
        double maxTicks = maxRotation * (ticksPerTurretRotation / 360.0);

        turretTarget = Math.max(minTicks, Math.min(maxTicks, turretTarget));

        double power = pid.getOutput(turret.motor.getCurrentPosition(), turretTarget);
        turret.queue_power(power);
    }

    // ------------------------
    // FLYWHEEL / HOOD (unchanged but cleaned)
    // ------------------------

    private void setFlyPID(double p, double i, double d) {
        fly.setVelocityPIDFCoefficients(p, i, d, 0);
    }

    private void computeFlySpeed() {
        // implement formula here
    }

    private void computeHoodAngle() {
        // implement formula here
    }

    private void computeDistance() {
        Pose pose = robot.follower.getPose();
        distance = robot.isRed() ?
                redGoal.distanceFrom(pose) :
                blueGoal.distanceFrom(pose);
    }

    private void updateQUS() {
        turret.update();
        hood1.update();
        hood2.update();
        kicker.update();
        safety.update();
    }

    // ------------------------
    // CONTROL COMMANDS
    // ------------------------

    public void launch() {
        if (turretError < turretTolerance && flyError < flyTolerance) {
            if (safety.servo.getPosition() == safetyOff) {
                canSpin = false;
                kicker.queue_position(kickPos);
            }
        }
    }

    public void resetKicker() {
        kicker.queue_position(0);
        canSpin = true;
    }

    public void aim() { aim = true; }
    public void stopAim() { aim = false; }

    public void disengageSafety() {
        safety.queue_position(safetyOff);
    }

    public void engageSafety() {
        safety.queue_position(safetyOn);
    }

    @Override
    public void shutdown() {
        super.shutdown();
        fly.setVelocity(0);
        engageSafety();
    }
}
