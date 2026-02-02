package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.examples;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;

/**
 * SimpleBallPickup - Minimal example of using the dual camera ball pickup system
 * 
 * This is the simplest possible autonomous that:
 * 1. Waits for AprilTag localization
 * 2. Searches for a ball
 * 3. Moves to the ball
 * 4. Picks it up
 * 
 * Perfect for understanding the system basics before using advanced features
 */
@Autonomous(name = "Simple Ball Pickup Example", group = "Examples")
public class SimpleBallPickupExample extends LiveAutoBase {
    private DualCamera camera;
    private ElapsedTime timer;
    private boolean hasSearched = false;
    private boolean hasLocalized = false;
    private BallTarget targetBall = null;

    @Override
    public void on_init() {
        telemetry.addLine("Initializing camera...");
        
        // Create and register camera
        camera = new DualCamera(robot);
        camera.registerHardware(hardwareMap);
        robot.registerComponent(camera);
        
        // Configure for yellow balls
        camera.setBallColorRange(15, 40, 100, 255, 100, 255);
        camera.setMinBallArea(150);
        
        telemetry.addLine("✓ Camera initialized");
        telemetry.update();
    }

    @Override
    public void on_start() {
        telemetry.addLine("Starting autonomous...");
        
        // Startup camera
        camera.startup();
        
        // Set starting position (adjust for your field)
        robot.follower.setPose(new Pose(0, 0, 0));
        
        timer = new ElapsedTime();
        telemetry.update();
    }

    @Override
    public void on_loop() {
        // Update camera - THIS IS CRITICAL
        robot.update();
        camera.update(this);

        // Step 1: Wait for AprilTag localization
        if (!hasLocalized) {
            if (camera.hasAprilTagLock()) {
                Pose detectedPose = camera.getPose();
                if (detectedPose != null) {
                    robot.follower.setPose(detectedPose);
                    hasLocalized = true;
                    timer.reset();
                    telemetry.addLine("✓ Localized with AprilTag");
                }
            } else {
                telemetry.addLine("⏳ Searching for AprilTag...");
            }
            telemetry.update();
            return;
        }

        // Step 2: Search for ball (first 3 seconds)
        if (!hasSearched) {
            if (camera.hasBallDetections()) {
                targetBall = camera.getClosestBall();
                hasSearched = true;
                timer.reset();
                telemetry.addLine("✓ Ball found!");
                telemetry.update();
                return;
            }

            if (timer.seconds() > 3.0) {
                // No ball found, auto-complete
                telemetry.addLine("✗ No ball found - mission complete");
                telemetry.update();
                return;
            }

            telemetry.addLine("🔍 Searching for ball...");
            telemetry.update();
            return;
        }

        // Step 3: Move to ball
        if (targetBall != null && !robot.follower.isBusy()) {
            Pose currentPose = robot.follower.getPose();
            Pose ballPose = targetBall.toPose();

            // Create simple linear path to ball
            var path = robot.follower.pathBuilder()
                    .addPath(new com.pedropathing.geometry.BezierLine(
                            currentPose,
                            ballPose
                    ))
                    .setLinearHeadingInterpolation(
                            currentPose.getHeading(),
                            currentPose.getHeading()
                    )
                    .build();

            robot.follower.followPath(path, true);
            telemetry.addLine("→ Moving to ball...");
            telemetry.update();
        }

        // Step 4: Pickup when reached
        if (robot.follower.isBusy() == false && timer.seconds() > 0.5) {
            if (timer.seconds() < 2.0) {
                // Run intake to pickup ball
                robot.intake.setPower(1.0);
                telemetry.addLine("↓ Picking up ball...");
            } else {
                // Stop intake and finish
                robot.intake.stop();
                telemetry.addLine("✓ Mission complete!");
                telemetry.update();
                return;
            }
        }

        telemetry.update();
    }

    @Override
    public void on_stop() {
        robot.intake.stop();
        camera.shutdown();
    }
}
