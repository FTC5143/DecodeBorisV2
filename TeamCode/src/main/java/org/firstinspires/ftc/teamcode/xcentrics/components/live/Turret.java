package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
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
  public static double flyP = 0, flyI = 0, flyD = 0, flyF = 0;
  public static com.qualcomm.robotcore.hardware.PIDFCoefficients flyPidCoef = new com.qualcomm.robotcore.hardware.PIDFCoefficients(10,0,0,22.9);


    // ------------------------------
    // Shooter State
    // ------------------------------
    public static boolean shooterReady = false;
    public static boolean velocityReady = false;

    // ------------------------------
    // Turret Encoder Configuration
    // ------------------------------

    public static double ticksPerTurretRotation = 4839.3;      // encoder ticks per 360°
    public static double turretHeading;               // current turret heading (deg)
    public static double turretTarget;                // target turret heading (ticks)

    // ------------------------------
    // Other Servo Positions
    // ------------------------------
    //public static boolean aim = false;               // is turret tracking target     // flywheel allowed to spin
   // public static Pose testPose = new Pose(9,9,Math.toRadians(0));
    //public static  double power = 0;
    public static PIDFCoefficients turretPIDCoef = new PIDFCoefficients(0.01,0,0,0);
    public static double targetVelocity = 1300;
}
@Configurable

public class Turret extends Component {
    public static double a = 0.7,b =0 ,c = 0.0;

    // ------------------------------
    // Hardware
    // ------------------------------
    public DcMotorEx fly1;
    private DcMotorQUS turret;
    public double turretOffset = 0;            // calibration offset
    private ServoQUS hood1, kicker;
    private final PIDFController turretPID = new PIDFController(turretPIDCoef);

    private final LiveRobot robot;             // reference to robot

    private final Pose redGoal = new Pose(131,137), blueGoal = new Pose(12,136);      // goal positions
    public static double distance;             // distance to goal
    public static boolean autoAim = true;
    public static boolean resetEncoder = false;
    public static double servoPos = 0;

    {
        name = "turret";
    }

    public Turret(LiveRobot robot) {
        super(robot);
        this.robot = robot;
    }

    // ------------------------------
    // Hardware Registration
    // ------------------------------
    @Override
    public void registerHardware(HardwareMap map) {
        super.registerHardware(map);

        fly1 = map.get(DcMotorEx.class, "fly1");
        turret = new DcMotorQUS(map.get(DcMotorEx.class, "turret"));
        turret.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        hood1 = new ServoQUS(map.get(Servo.class, "H1"));
        kicker = new ServoQUS(map.get(Servo.class, "kicker"));
    }

    // ------------------------------
    // Initialization
    // ------------------------------
    @Override
    public void startup() {
        super.startup();


        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flyPidCoef);
        kicker.queue_position(1);

        if(LiveRobot.isAuto) {
            turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretPID.setCoefficients(turretPIDCoef);
        turret.motor.setPower(0);
        hood1.queue_position(0.5);
        updateAll();
    }

    // ------------------------------
    // Main Update Loop
    // ------------------------------
    @Override
    public void update(OpMode opMode) {
        super.update(opMode);

        turretPID.setCoefficients(turretPIDCoef);
 //       fly1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flyPidCoef);

        updateTurretHeading();  // update turret heading relative to robot
        computeDistance(); //see how far we are
        computeTurretTarget();


            if(resetEncoder){
                resetEncoder();
                resetEncoder = false;
            }
            computeHoodAngle();

        if (autoAim) {
            turret.queue_power(turretPID.run());
           computeFlySpeed();
        } else {
            turret.queue_power(0);
            fly1.setVelocity(0);
        }

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

        addData("Turret Heading", turretHeading);
        addData("TargetTicks", turretTarget);
        addData("TurretTicks",turret.motor.getCurrentPosition());
        addData("Distance", distance);
        addData("Fly1Vel", fly1.getVelocity());
        addData("FlyTargetVel",targetVelocity);
        addData("autoAim:", autoAim);
        addData("FlyError",Math.abs(fly1.getVelocity() - targetVelocity));
        addData("turretOffset",turretOffset);


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
        double minTicks = -3951;
        double maxTicks = 2119;

        turretTarget = Math.max(minTicks, Math.min(maxTicks, rawTicks));
        turretPID.setTargetPosition(turretTarget);
        turretPID.updatePosition(turret.motor.getCurrentPosition());
        shooterReady = (turretPID.getError() <= 10);


    }

    // ------------------------------
    // Hood / Flywheel Physics
    // ------------------------------
    private void computeHoodAngle() {
        servoPos = a + (b * distance) + (c * c * distance);
        hood1.queue_position(servoPos);
    }
    private void computeFlySpeed() {
        fly1.setVelocity(targetVelocity);
        velocityReady = Math.abs(fly1.getVelocity() - targetVelocity) <= 100;
    }
    public void resetEncoder(){
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    // ------------------------------
    // Launch / Kicker
    // ------------------------------
    public void launch() {
        kicker.queue_position(0);
        updateAll();
        halt(0.5);
        kicker.queue_position(0.9);
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
        kicker.update();
    }
    @Override
    public void shutdown() {
        super.shutdown();
    }
    public Runnable aim(){
        return () -> autoAim = true;
    }
    public Runnable stopAim(){
        return () -> autoAim = false;
    }
    public void spinUp(){
        targetVelocity += 100;
    }
    public void spinDown(){
        targetVelocity -=100;
    }
}
