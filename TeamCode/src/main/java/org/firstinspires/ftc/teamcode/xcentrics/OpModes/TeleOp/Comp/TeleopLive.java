package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.Comp;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.geometry.Pose;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp.LiveTeleopBase;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;


@TeleOp(name = "TeleOp")
public class TeleopLive extends LiveTeleopBase {

    //small triangle points

    //robot pose
    private GamepadManager g1 = PanelsGamepad.INSTANCE.getFirstManager();
    private GamepadManager g2 = PanelsGamepad.INSTANCE.getSecondManager();

    private Timer opMode = new Timer();
    //private PanelsGamepad g2 = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(OpModeInternal.this.gamepad2);
    //triangle Checker
    @Override
    public void on_init() {
        robot.follower.setStartingPose(robot.getLastPose());
    }

    @Override
    public void on_start() {
        opMode.resetTimer();
        robot.follower.startTeleOpDrive();
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        gamepad1 = g1.asCombinedFTCGamepad(gamepad1);
        gamepad2 = g2.asCombinedFTCGamepad(gamepad2);


    }
}
