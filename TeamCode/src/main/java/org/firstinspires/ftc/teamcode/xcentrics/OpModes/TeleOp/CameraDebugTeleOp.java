package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;

/**
 * CameraDebugTeleOp - Debug and tune the dual camera vision system
 * 
 * Controls:
 * - Left Stick: Manual robot movement
 * - Right Stick: Rotation
 * - A: Intake
 * - B: Stop intake
 * - X: Toggle ball detection color range
 * - Y: Display detected balls
 * - DPAD UP/DOWN: Adjust HSV threshold
 * - DPAD LEFT/RIGHT: Adjust min ball area
 */
@TeleOp(name = "Camera Debug TeleOp", group = "Debug")
public class CameraDebugTeleOp extends OpMode {
    private LiveRobot robot;
    private DualCamera dualCamera;

    // HSV adjustment
    private int hMin = 15;
    private int hMax = 40;
    private int sMin = 100;
    private int sMax = 255;
    private int vMin = 100;
    private int vMax = 255;
    private double minBallArea = 150;

    private boolean prevX = false;
    private int colorPreset = 0; // 0=Yellow, 1=Orange, 2=Red

    @Override
    public void init() {
        robot = new LiveRobot(this);
        dualCamera = new DualCamera(robot);
        dualCamera.registerHardware(hardwareMap);
        robot.registerComponent(dualCamera);

        // Set initial color range for yellow balls
        setColorPreset(0);

        telemetry.addLine("Camera Debug TeleOp Initialized");
        telemetry.addLine("X: Change color preset");
        telemetry.addLine("DPAD UP/DOWN: Adjust V (brightness)");
        telemetry.addLine("DPAD LEFT/RIGHT: Adjust min area");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.update();
    }

    @Override
    public void start() {
        robot.startup();
        robot.turret.startup();
        robot.intake.startup();
        dualCamera.startup();
    }

    @Override
    public void loop() {
        robot.update();
        dualCamera.update(this);

        // Manual drive control
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        // Intake control
        if (gamepad1.a) {
            robot.intake.setPower(1.0);
        } else if (gamepad1.b) {
            robot.intake.stop();
        }

        // Color preset selection
        if (gamepad1.x && !prevX) {
            colorPreset = (colorPreset + 1) % 3;
            setColorPreset(colorPreset);
        }
        prevX = gamepad1.x;

        // HSV fine-tuning
        if (gamepad1.dpad_up) {
            vMax = Math.min(255, vMax + 2);
            dualCamera.setBallColorRange(hMin, hMax, sMin, sMax, vMin, vMax);
        }
        if (gamepad1.dpad_down) {
            vMin = Math.max(0, vMin - 2);
            dualCamera.setBallColorRange(hMin, hMax, sMin, sMax, vMin, vMax);
        }

        // Min area adjustment
        if (gamepad1.dpad_left) {
            minBallArea = Math.max(50, minBallArea - 10);
            dualCamera.setMinBallArea(minBallArea);
        }
        if (gamepad1.dpad_right) {
            minBallArea += 10;
            dualCamera.setMinBallArea(minBallArea);
        }

        updateTelemetry();
    }

    @Override
    public void stop() {
        robot.shutdown();
        dualCamera.shutdown();
    }

    private void setColorPreset(int preset) {
        switch (preset) {
            case 0: // Yellow
                hMin = 15;
                hMax = 40;
                sMin = 100;
                sMax = 255;
                vMin = 100;
                vMax = 255;
                break;
            case 1: // Orange
                hMin = 5;
                hMax = 25;
                sMin = 130;
                sMax = 255;
                vMin = 100;
                vMax = 255;
                break;
            case 2: // Red
                hMin = 0;
                hMax = 15;
                sMin = 100;
                sMax = 255;
                vMin = 100;
                vMax = 255;
                break;
        }
        dualCamera.setBallColorRange(hMin, hMax, sMin, sMax, vMin, vMax);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== DUAL CAMERA DEBUG ===\n");

        // AprilTag status
        telemetry.addLine("AprilTag Detection:");
        telemetry.addData("  Status", dualCamera.hasAprilTagLock() ? "✓ Locked" : "✗ No Lock");
        telemetry.addData("  Tags Detected", dualCamera.currentDetections.size());

        Pose detectedPose = dualCamera.getPose();
        if (detectedPose != null) {
            telemetry.addData("  Pose", "X:%.1f Y:%.1f H:%.1f°",
                    detectedPose.getX(),
                    detectedPose.getY(),
                    Math.toDegrees(detectedPose.getHeading()));
        }

        // Ball detection status
        telemetry.addLine("\nBall Detection:");
        telemetry.addData("  Balls Found", dualCamera.getBallCount());

        if (dualCamera.hasBallDetections()) {
            for (int i = 0; i < dualCamera.getDetectedBalls().size(); i++) {
                BallTarget ball = dualCamera.getDetectedBalls().get(i);
                telemetry.addData(String.format("  Ball %d", i + 1),
                        "X:%.1f Y:%.1f Conf:%.2f",
                        ball.fieldX, ball.fieldY, ball.confidence);
            }

            BallTarget closest = dualCamera.getClosestBall();
            if (closest != null) {
                telemetry.addLine(String.format("  Closest: (%.1f, %.1f) conf=%.2f",
                        closest.fieldX, closest.fieldY, closest.confidence));
            }
        }

        // HSV Configuration
        telemetry.addLine("\nColor Configuration:");
        String[] presets = {"Yellow", "Orange", "Red"};
        telemetry.addData("  Preset (X to cycle)", presets[colorPreset]);
        telemetry.addData("  H Range", hMin + " - " + hMax);
        telemetry.addData("  S Range", sMin + " - " + sMax);
        telemetry.addData("  V Range (↑↓)", vMin + " - " + vMax);
        telemetry.addData("  Min Area (←→)", "%.0f", minBallArea);

        telemetry.update();
    }
}
