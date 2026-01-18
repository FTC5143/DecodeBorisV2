package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.basic;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Basic")
public class basic extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Fl = hardwareMap.get(DcMotor.class,"FL"),Bl = hardwareMap.get(DcMotor.class,"BL"),Br = hardwareMap.get(DcMotor.class,"BR"),Fr = hardwareMap.get(DcMotor.class,"FR");

        waitForStart();
        Fl.setPower(1);
        Bl.setPower(1);
        Br.setPower(-1);
        Fr.setPower(-1);
        sleep(500);
        Fl.setPower(0);
        Bl.setPower(0);
        Br.setPower(0);
        Fr.setPower(0);
    }


}
