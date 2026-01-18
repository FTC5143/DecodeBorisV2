package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.classes;

/// DONOT USE WITHOUT COMMENTING OUT d.p();
@SuppressWarnings({"ALL", "unused"})
@Disabled
///  DONOT USE WITHOUT COMMENTINGT OUT d.p();
@Autonomous(name = "Basic")
public class basic extends LinearOpMode {
    private final classes d = new classes();

    @Override
    public void runOpMode() {
        d.p();
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
