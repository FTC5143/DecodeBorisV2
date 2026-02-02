package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;

/**
 * BallPickupAuto - Autonomous ball pickup with dual camera vision
 * 
 * Strategy:
 * 1. Use AprilTag localization for accurate pose estimation
 * 2. Use ball detection to find nearby balls
 * 3. Navigate to detected balls using Pedro Pathing
 * 4. Execute pickup sequence
 * 5. Return to scoring position
 * 
 * The dual camera system runs concurrently:
 * - Camera 1: Continuous AprilTag detection for pose
 * - Camera 2: Continuous ball detection for target acquisition
 */
@Autonomous(name = "Ball Pickup Auto", group = "Competition")
public class BallPickupAuto extends LinearOpMode {
    private LiveRobot robot;
    private DualCamera dualCamera;
    private ElapsedTime runtime;

    // State machine
    private enum State {
        INITIALIZE,
        LOCALIZE,
        SEARCH_FOR_BALL,
        APPROACH_BALL,
        PICKUP,
        RETURN_TO_START,
        COMPLETE
    }

    private State currentState = State.INITIALIZE;
    private BallTarget currentTarget = null;
    private Pose startPose;
    private int ballsPickedUp = 0;
    private final int MAX_BALLS = 3;
    private final double BALL_DETECTION_TIMEOUT = 5.0; // seconds

    @Override
    public void runOpMode() {
        // Initialize robot
        robot = new LiveRobot(this);
        runtime = new ElapsedTime();

        // Initialize dual camera system
        dualCamera = new DualCamera(robot);
        dualCamera.registerHardware(hardwareMap);
        robot.registerComponent(dualCamera);

        // Configure ball detection for yellow/orange balls
        // HSV range: Yellow-Orange
        dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);
        dualCamera.setMinBallArea(100);

        // Startup components
        robot.startup();
        robot.turret.startup();
        robot.intake.startup();
        dualCamera.startup();

        // Wait for start
        telemetry.addData("Status", "Ready for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        startPose = new Pose(0, 0, 0); // Set to your starting position
        runtime.reset();

        // Main control loop
        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
            dualCamera.update(this);
            updateTelemetry();

            switch (currentState) {
                case INITIALIZE:
                    handleInitialize();
                    break;
                case LOCALIZE:
                    handleLocalize();
                    break;
                case SEARCH_FOR_BALL:
                    handleSearchForBall();
                    break;
                case APPROACH_BALL:
                    handleApproachBall();
                    break;
                case PICKUP:
                    handlePickup();
                    break;
                case RETURN_TO_START:
                    handleReturnToStart();
                    break;
                case COMPLETE:
                    robot.follower.breakFollowing();
                    requestOpModeStop();
                    break;
            }

            idle();
        }

        robot.shutdown();
    }

    private void handleInitialize() {
        // Set initial conditions
        robot.intake.stop();
        robot.turret.reset();
        telemetry.addData("State", "Initializing");
        currentState = State.LOCALIZE;
    }

    private void handleLocalize() {
        // Wait for AprilTag lock
        if (dualCamera.hasAprilTagLock()) {
            Pose detectedPose = dualCamera.getPose();
            if (detectedPose != null) {
                robot.follower.setPose(detectedPose);
                telemetry.addData("Pose Locked", "%.2f, %.2f, %.2f°",
                        detectedPose.getX(), detectedPose.getY(),
                        Math.toDegrees(detectedPose.getHeading()));

                // Move to search state
                currentState = State.SEARCH_FOR_BALL;
                runtime.reset();
            }
        } else {
            telemetry.addData("Status", "Waiting for AprilTag lock...");
        }
    }

    private void handleSearchForBall() {
        // Search for balls
        if (dualCamera.hasBallDetections()) {
            currentTarget = dualCamera.getClosestBall();
            if (currentTarget != null) {
                telemetry.addData("Ball Found", currentTarget.toString());
                currentState = State.APPROACH_BALL;
                runtime.reset();
            }
        } else {
            if (runtime.seconds() > BALL_DETECTION_TIMEOUT) {
                // No balls found, move to complete
                telemetry.addData("Status", "No balls detected");
                currentState = State.COMPLETE;
            } else {
                // Continue searching - can implement search pattern here
                telemetry.addData("Status", "Searching for balls...");
                telemetry.addData("Search Time", "%.1f", runtime.seconds());
            }
        }
    }

    private void handleApproachBall() {
        if (currentTarget == null) {
            currentState = State.SEARCH_FOR_BALL;
            return;
        }

        // Create path to ball
        Pose currentPose = robot.follower.getPose();
        Pose ballPose = currentTarget.toPose();

        // Add orientation towards ball
        double angle = Math.atan2(
                ballPose.getY() - currentPose.getY(),
                ballPose.getX() - currentPose.getX()
        );
        ballPose = new Pose(ballPose.getX(), ballPose.getY(), angle);

        // Build and follow path
        PathChain pathToBall = robot.follower.pathBuilder()
                .addPath(new com.pedropathing.geometry.BezierLine(currentPose, ballPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), ballPose.getHeading())
                .build();

        robot.follower.followPath(pathToBall, true);

        // Check if reached ball
        if (robot.follower.getCurrentTValue() > 0.9 || !robot.follower.isBusy()) {
            currentState = State.PICKUP;
            runtime.reset();
        }

        telemetry.addData("Approaching", currentTarget.toString());
    }

    private void handlePickup() {
        // Execute pickup sequence
        if (runtime.seconds() < 0.5) {
            // Start intake
            robot.intake.setPower(1.0);
        } else if (runtime.seconds() < 2.0) {
            // Continue intake
            robot.intake.setPower(1.0);
        } else {
            // Pickup complete
            robot.intake.stop();
            ballsPickedUp++;
            currentTarget = null;

            telemetry.addData("Ball Picked Up", ballsPickedUp);

            // Check if we should get more balls
            if (ballsPickedUp >= MAX_BALLS) {
                currentState = State.RETURN_TO_START;
            } else {
                currentState = State.SEARCH_FOR_BALL;
            }
            runtime.reset();
        }
    }

    private void handleReturnToStart() {
        // Path back to starting position
        Pose currentPose = robot.follower.getPose();

        PathChain pathToStart = robot.follower.pathBuilder()
                .addPath(new com.pedropathing.geometry.BezierLine(currentPose, startPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), startPose.getHeading())
                .build();

        robot.follower.followPath(pathToStart, true);

        if (!robot.follower.isBusy()) {
            currentState = State.COMPLETE;
        }

        telemetry.addData("Returning to start", "%.2f%%", robot.follower.getCurrentTValue() * 100);
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Balls Picked Up", ballsPickedUp + "/" + MAX_BALLS);
        telemetry.addData("Ball Count", dualCamera.getBallCount());
        telemetry.addData("AprilTag Lock", dualCamera.hasAprilTagLock());
        telemetry.addData("Runtime", "%.2f s", runtime.seconds());

        Pose currentPose = robot.follower.getPose();
        if (currentPose != null) {
            telemetry.addData("Current Pose",
                    "X: %.2f, Y: %.2f, Heading: %.2f°",
                    currentPose.getX(),
                    currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));
        }

        dualCamera.updateTelemetry(telemetry);
        telemetry.update();
    }
}
