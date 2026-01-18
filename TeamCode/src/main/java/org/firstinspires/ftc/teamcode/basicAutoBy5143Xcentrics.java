package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is a basic auto where the robot moves a direction for a set amount of time then stops
 * @author Alex Stephens - 5143  Xcentrics
 */
@Autonomous(name = "Basic Auto (By 5143 Xcentrics)")
public class basicAutoBy5143Xcentrics extends LinearOpMode {
    private DcMotor frontLeftMotor,frontRightMotor,backRightMotor,backLeftMotor;
    private final long wait = 1000;

    /// to do:
    /// Match motor variables with hardware map
    /// make motors run the same way
    @Override
    public void runOpMode(){

        //assign the motor variable to the motor on the robot
        frontLeftMotor = hardwareMap.get(DcMotor.class,"");
        frontRightMotor = hardwareMap.get(DcMotor.class,"");
        backRightMotor = hardwareMap.get(DcMotor.class,"");
        backLeftMotor = hardwareMap.get(DcMotor.class,"");

        //reverse the right side so the motors spin in the same direction
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addLine("Initilized");
        telemetry.update();
        telemetry.update();

        //waits for someone to press start on the driver hub
        waitForStart();

        //set the motors to run at full power
        move(1);


        //telemetry to know if it is moveing to be displayed by the driver hub
        telemetry.addLine("Moving");
        telemetry.update();


        //wait
        sleep(wait);

        //stop motors
        move(0);

        //telemetry to display on the driver hub so we know it is stopped
        telemetry.addLine("Stopped");
        telemetry.update();

    }

    //This is a method, every time it is called the code inside runs
    /**
     *
     * This method will set the power of all motors to the givin input
     * @param power the power you want to set all the motors to
     */
    private void move(double power){
        frontRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }
}
