package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.examples;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;
import org.firstinspires.ftc.teamcode.xcentrics.vision.CameraOffsetConfig;

/**
 * HuskyLensBallPickupExample - Ball pickup with Husky Lens camera
 * 
 * This example demonstrates:
 * 1. Using Husky Lens for ball detection
 * 2. Handling side-mounted camera with offset
 * 3. Accurate ball tracking with camera position compensation
 * 
 * Setup:
 * - Mount Husky Lens camera to side of robot (e.g., 6 inches right)
 * - Configure in on_init() with useCameraRightMounted(6.0)
 * - Husky Lens must have ball detection algorithm pre-configured
 */
@Autonomous(name = "Husky Lens Ball Pickup Example", group = "Examples")
public class HuskyLensBallPickupExample extends LiveAutoBase {
    private DualCamera camera;
    private ElapsedTime timer;
    private boolean hasSearched = false;
    private boolean hasLocalized = false;
    private BallTarget targetBall = null;

    @Override
    public void on_init() {
        telemetry.addLine("Initializing Husky Lens camera...");
        
        // Create and register camera
        camera = new DualCamera(robot);
        camera.registerHardware(hardwareMap);
        robot.registerComponent(camera);
        
        // IMPORTANT: Enable Husky Lens mode instead of HSV
        camera.useHuskyLensDetection();
        
        // Configure camera mounting position
        // Example: Camera mounted 6 inches to the right of robot center
        // This offset is automatically applied to all ball position calculations
        camera.useCameraRightMounted(6.0);
        
        // Alternative configurations:
        // camera.useCameraLeftMounted(6.0);        // 6 inches left
        // camera.useCameraForwardMounted(8.0);     // 8 inches forward
        // camera.useCameraCenterMounted();         // Center mounted (default)
        
        // Or manual configuration:
        // CameraOffsetConfig config = new CameraOffsetConfig();
        // config.setCameraPosition(-6.0, 0, 12.0);  // Left 6", no forward offset, 12" height
        // camera.setCameraOffsetConfig(config);
        
        telemetry.addLine("✓ Husky Lens initialized with side-mount offset");
        telemetry.addLine("  Camera position: 6 inches RIGHT of center");
        telemetry.update();
    }

    @Override
    public void on_start() {
        telemetry.addLine("Starting Husky Lens autonomous...");
        
        // Startup camera
        camera.startup();
        
        // Set starting position (adjust for your field)
        robot.follower.setPose(new Pose(0, 0, 0));
        
        timer = new ElapsedTime();
        telemetry.update();
    }

    @Override
    public void on_loop() {
        // Update camera - CRITICAL for Husky Lens updates
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
                telemetry.addLine(String.format("  Position: X=%.1f Y=%.1f",
                        targetBall.fieldX, targetBall.fieldY));
                telemetry.update();
                return;
            }

            if (timer.seconds() > 3.0) {
                // No ball found, mission complete
                telemetry.addLine("✗ No ball found - mission complete");
                telemetry.update();
                return;
            }

            telemetry.addLine("🔍 Searching for ball with Husky Lens...");
            telemetry.addData("  Balls Detected", camera.getBallCount());
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
            telemetry.addLine("→ Moving to ball (offset-compensated position)...");
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
