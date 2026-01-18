package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@SuppressWarnings({"ALL", "unused"})
@TeleOp(name = "Debug")
public class DebugTeleOp extends LiveTeleopBase{
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
