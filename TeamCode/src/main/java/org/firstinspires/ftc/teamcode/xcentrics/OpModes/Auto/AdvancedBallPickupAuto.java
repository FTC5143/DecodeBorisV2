package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.util.navigation.BallPathPlanner;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;

import java.util.ArrayList;
import java.util.List;

/**
 * AdvancedBallPickupAuto - Multi-ball pickup with optimized pathfinding
 * 
 * Enhanced features:
 * - Dual camera concurrent processing (AprilTag + Ball detection)
 * - Optimized multi-ball pickup sequence (nearest neighbor sorting)
 * - Smooth bezier curve pathfinding
 * - Continuous pose estimation during navigation
 * - Performance telemetry
 */
@Autonomous(name = "Advanced Ball Pickup Auto", group = "Competition")
public class AdvancedBallPickupAuto extends LiveAutoBase {
    private DualCamera dualCamera;
    private BallPathPlanner pathPlanner;
    private ElapsedTime stateTimer;

    private enum State {
        INITIALIZE,
        LOCALIZE,
        DETECT_BALLS,
        PLAN_ROUTE,
        EXECUTE_PICKUPS,
        SCORE,
        COMPLETE
    }

    private State currentState = State.INITIALIZE;
    private List<BallTarget> detectedBalls = new ArrayList<>();
    private int ballPickupIndex = 0;
    private int ballsCollected = 0;
    private final int MAX_BALLS_TO_COLLECT = 3;
    private final double MAX_DETECTION_TIME = 4.0; // seconds

    // Starting positions (configure for your field)
    private final Pose BLUE_START = new Pose(27.5, 131.6, Math.toRadians(144));
    private final Pose RED_START = new Pose(-27.5, 131.6, Math.toRadians(36));
    private final Pose SCORING_POSITION = new Pose(50, 110, 0);

    @Override
    public void on_init() {
        // Initialize camera system
        dualCamera = new DualCamera(robot);
        dualCamera.registerHardware(hardwareMap);
        robot.registerComponent(dualCamera);

        // Configure ball detection (yellow/orange balls)
        dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);
        dualCamera.setMinBallArea(150);

        // Initialize path planner
        pathPlanner = new BallPathPlanner(robot.follower);
        pathPlanner.setStartPose(robot.isRed() ? RED_START : BLUE_START);
        pathPlanner.setScoringPose(SCORING_POSITION);
        pathPlanner.setApproachDistance(5.0); // inches

        stateTimer = new ElapsedTime();
        telemetry.addData("Status", "Initialized with dual camera system");
    }

    @Override
    public void on_start() {
        // Startup components
        dualCamera.startup();
        robot.follower.setPose(robot.isRed() ? RED_START : BLUE_START);
        stateTimer.reset();

        telemetry.addData("Status", "Started - waiting for AprilTag lock");
        telemetry.update();
    }

    @Override
    public void on_loop() {
        // Update camera systems
        robot.update();
        dualCamera.update(this);

        switch (currentState) {
            case INITIALIZE:
                currentState = State.LOCALIZE;
                break;

            case LOCALIZE:
                handleLocalize();
                break;

            case DETECT_BALLS:
                handleDetectBalls();
                break;

            case PLAN_ROUTE:
                handlePlanRoute();
                break;

            case EXECUTE_PICKUPS:
                handleExecutePickups();
                break;

            case SCORE:
                handleScore();
                break;

            case COMPLETE:
                robot.follower.breakFollowing();
                break;
        }

        updateDebugTelemetry();
    }

    @Override
    public void on_stop() {
        dualCamera.shutdown();
    }

    private void handleLocalize() {
        // Wait for AprilTag lock to establish initial pose
        if (dualCamera.hasAprilTagLock()) {
            Pose detectedPose = dualCamera.getPose();
            if (detectedPose != null) {
                robot.follower.setPose(detectedPose);
                telemetry.addLine("✓ Pose locked from AprilTag");
                currentState = State.DETECT_BALLS;
                stateTimer.reset();
            }
        } else {
            telemetry.addLine("⏳ Waiting for AprilTag lock...");
        }
    }

    private void handleDetectBalls() {
        // Search for balls with timeout
        if (dualCamera.hasBallDetections()) {
            detectedBalls = dualCamera.getDetectedBalls();
            telemetry.addData("Balls Found", detectedBalls.size());

            if (detectedBalls.size() > 0) {
                currentState = State.PLAN_ROUTE;
                stateTimer.reset();
            }
        } else {
            if (stateTimer.seconds() > MAX_DETECTION_TIME) {
                // No balls found, move to scoring
                telemetry.addLine("⚠ No balls detected - moving to score");
                currentState = State.SCORE;
            } else {
                telemetry.addLine("🔍 Searching for balls...");
            }
        }
    }

    private void handlePlanRoute() {
        // Create optimized pickup path
        telemetry.addLine("📍 Planning optimal route...");

        // Calculate path efficiency
        Pose currentPose = robot.follower.getPose();
        double totalDistance = pathPlanner.calculateTotalDistance(currentPose, detectedBalls);
        double estimatedTime = pathPlanner.estimatePickupTime(currentPose, detectedBalls, 30);

        telemetry.addData("Route Distance", "%.1f in", totalDistance);
        telemetry.addData("Est. Time", "%.1f s", estimatedTime);

        currentState = State.EXECUTE_PICKUPS;
        ballPickupIndex = 0;
        stateTimer.reset();
    }

    private void handleExecutePickups() {
        Pose currentPose = robot.follower.getPose();

        // Check if we've collected enough balls
        if (ballPickupIndex >= detectedBalls.size() || ballsCollected >= MAX_BALLS_TO_COLLECT) {
            currentState = State.SCORE;
            stateTimer.reset();
            return;
        }

        BallTarget targetBall = detectedBalls.get(ballPickupIndex);

        // Create curved approach path for smooth pickup
        if (!robot.follower.isBusy()) {
            PathChain pickupPath = pathPlanner.createCurvedBallApproach(currentPose, targetBall);
            robot.follower.followPath(pickupPath, true);
            telemetry.addLine(String.format("→ Moving to ball %d/%d", ballPickupIndex + 1, detectedBalls.size()));
        }

        // Check if reached ball and execute pickup
        if (!robot.follower.isBusy()) {
            // Pickup sequence
            if (stateTimer.seconds() < 0.2) {
                robot.intake.setPower(1.0);
            } else if (stateTimer.seconds() < 1.5) {
                robot.intake.setPower(1.0);
            } else {
                // Ball collected
                robot.intake.stop();
                ballsCollected++;
                ballPickupIndex++;
                stateTimer.reset();

                if (ballPickupIndex < detectedBalls.size() && ballsCollected < MAX_BALLS_TO_COLLECT) {
                    // Get next ball
                    targetBall = detectedBalls.get(ballPickupIndex);
                    telemetry.addLine(String.format("✓ Ball %d collected", ballsCollected));
                }
            }
        }

        telemetry.addData("Balls Collected", ballsCollected);
        telemetry.addData("Pickup Progress", "%.0f%%", 
                (ballPickupIndex + stateTimer.seconds() / 1.5) / detectedBalls.size() * 100);
    }

    private void handleScore() {
        // Navigate to scoring position
        Pose currentPose = robot.follower.getPose();

        if (!robot.follower.isBusy()) {
            PathChain scoreChain = pathPlanner.createReturnPath(currentPose);
            robot.follower.followPath(scoreChain, true);
            telemetry.addLine("→ Moving to scoring position");
        }

        if (!robot.follower.isBusy() && stateTimer.seconds() > 0.5) {
            // Execute scoring sequence
            robot.intake.setPower(-0.5); // Reverse to score
            
            if (stateTimer.seconds() > 3.0) {
                robot.intake.stop();
                currentState = State.COMPLETE;
                telemetry.addLine("✓ Scoring complete");
            }
        }

        telemetry.addData("Scoring Progress", "%.0f%%", 
                Math.min(100, stateTimer.seconds() / 3.0 * 100));
    }

    private void updateDebugTelemetry() {
        telemetry.addLine("=== Ball Pickup Auto ===");
        telemetry.addData("State", currentState.toString());
        telemetry.addData("State Timer", "%.2f s", stateTimer.seconds());

        Pose currentPose = robot.follower.getPose();
        if (currentPose != null) {
            telemetry.addData("Position", "X:%.1f Y:%.1f", currentPose.getX(), currentPose.getY());
        }

        telemetry.addData("Follower Busy", robot.follower.isBusy());

        // Camera status
        telemetry.addLine("\n--- Vision Status ---");
        telemetry.addData("AprilTag Lock", dualCamera.hasAprilTagLock() ? "✓" : "✗");
        telemetry.addData("Balls Detected", dualCamera.getBallCount());
        telemetry.addData("Balls Collected", ballsCollected + "/" + MAX_BALLS_TO_COLLECT);

        telemetry.update();
    }
}
