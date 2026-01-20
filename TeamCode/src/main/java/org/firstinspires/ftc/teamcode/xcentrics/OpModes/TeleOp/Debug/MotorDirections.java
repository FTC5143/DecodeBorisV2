package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.Debug;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;


@Configurable
@TeleOp(name = "Motor Directions", group = "Teleop Test")
public class MotorDirections extends OpMode {
    
    public static double leftFrontMotorDirection = 0;
    
    public static double leftRearMotorDirection = 0;
    
    public static double rightFrontMotorDirection = 0;
    
    public static double rightRearMotorDirection = 0;
    
    public static double test = 1;

    //private MultipleTelemetry telemetryM;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftRear.setDirection(direction(leftRearMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightRear.setDirection(direction(rightRearMotorDirection));

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void loop() {
        leftFront.setDirection(direction(leftFrontMotorDirection));
        leftRear.setDirection(direction(leftRearMotorDirection));
        rightFront.setDirection(direction(rightFrontMotorDirection));
        rightRear.setDirection(direction(rightRearMotorDirection));

        if(gamepad1.a)
            leftFront.setPower(1);
        else
            leftFront.setPower(0);

        if(gamepad1.y)
            leftRear.setPower(1);
        else
            leftRear.setPower(0);

        if(gamepad1.b)
            rightFront.setPower(1);
        else
            rightFront.setPower(0);

        if(gamepad1.x)
            rightRear.setPower(1);
        else
            rightRear.setPower(0);

        telemetry.addLine("Press A to spin the left front motor at 100% power");
        telemetry.addLine("Press Y to spin the left rear motor at 100% power");
        telemetry.addLine("Press B to spin the right front motor at 100% power");
        telemetry.addLine("Press X to spin the right rear motor at 100% power");
        telemetry.addLine("Left Front Motor Direction: " + leftFrontMotorDirection);
        telemetry.addLine("Left Rear Motor Direction: "+ leftRearMotorDirection);
        telemetry.addLine("Right Front Motor Direction: "+ rightFrontMotorDirection);
        telemetry.addLine("Right Rear Motor Direction: "+ rightRearMotorDirection);
        telemetry.update();
    }

    public DcMotorSimple.Direction direction(double d) {
        if (d == 0)
            return DcMotorSimple.Direction.FORWARD;
        else
            return DcMotorSimple.Direction.REVERSE;
    }
}
