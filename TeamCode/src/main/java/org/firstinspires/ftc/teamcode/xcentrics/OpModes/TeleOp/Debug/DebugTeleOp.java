package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.LiveTeleopBase;


@TeleOp(name = "Debug")
public class DebugTeleOp extends LiveTeleopBase {
    @Override
    public void on_init() {

    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        robot.turret.update(this);
        robot.update();
    }
}
