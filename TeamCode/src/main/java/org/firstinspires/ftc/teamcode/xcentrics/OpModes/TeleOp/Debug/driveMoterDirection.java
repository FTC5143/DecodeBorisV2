package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorDirection")
public class driveMoterDirection extends OpMode {
    private DcMotor FL,BL,BR,FR;
    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class,"FL");
        BL = hardwareMap.get(DcMotor.class,"BL");
        BR = hardwareMap.get(DcMotor.class,"BR");
        FR = hardwareMap.get(DcMotor.class,"FR");
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            FL.setPower(1);
        }
        if(gamepad1.a){
            FR.setPower(1);
        }
        if(gamepad1.b){
            BL.setPower(1);
        }
        if(gamepad1.y){
            BR.setPower(1);
        }

    }
}
