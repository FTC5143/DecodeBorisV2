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
    private GamepadManager g1 = PanelsGamepad.INSTANCE.getFirstManager();
    private GamepadManager g2 = PanelsGamepad.INSTANCE.getSecondManager();
    private Timer opMode = new Timer();

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

        robot.follower.setTeleOpDrive(
                0 - gamepad1.left_stick_y,
                0 - gamepad1.left_stick_x,
                0 - gamepad1.right_stick_x
        );

        if(gamepad2.a){
            robot.shooter.spinGate();
        } else {
            robot.shooter.stopGate();
        }

        if(gamepad2.y){
            robot.shooter.spinUp();
        } else if (gamepad2.x) {
            robot.shooter.spinDown();
        }

        if(gamepad2.left_bumper){
            robot.intake.reverse();
        } else if (gamepad2.right_bumper) {
            robot.intake.intake();
        } else {
            robot.intake.stopIntake();
        }

    }
}
