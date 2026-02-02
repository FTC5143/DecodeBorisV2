package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import android.annotation.SuppressLint;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallDetectionProcessor;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;
import org.firstinspires.ftc.teamcode.xcentrics.vision.CameraOffsetConfig;
import org.firstinspires.ftc.teamcode.xcentrics.vision.HuskyLensBallDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * DualCamera - Advanced camera system with concurrent pipelines
 * 
 * Pipeline 1: AprilTag Detection for accurate pose estimation
 * Pipeline 2: Ball Detection using HSV processing OR Husky Lens
 * 
 * Features:
 * - Concurrent AprilTag and ball detection
 * - Support for Husky Lens camera for ball detection
 * - Camera offset compensation for side-mounted cameras
 * - Precise coordinate transformation with offset correction
 * 
 * Both pipelines run concurrently on separate vision portal threads
 */
public class DualCamera extends Component {
    private final Position cameraPosition = new Position(DistanceUnit.MM,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    // AprilTag Detection Pipeline
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal aprilTagPortal;
    public List<AprilTagDetection> currentDetections;

    // Ball Detection Pipeline - choose one
    private BallDetectionProcessor ballDetectionProcessor; // For webcam HSV detection
    private HuskyLensBallDetector huskyLensBallDetector; // For Husky Lens I2C detection
    private VisionPortal ballDetectionPortal;
    private List<BallTarget> detectedBallTargets;

    // Camera configuration and offset
    private CameraOffsetConfig cameraOffsetConfig;
    private boolean useHuskyLens = true;

    // Camera calibration parameters (adjust based on your camera)
    private static final double FOCAL_LENGTH_X = 578.272; // pixels
    private static final double FOCAL_LENGTH_Y = 578.272; // pixels
    private static final double PRINCIPAL_POINT_X = 402.145; // pixels
    private static final double PRINCIPAL_POINT_Y = 221.506; // pixels
    private static final double CAMERA_HEIGHT_INCHES = 12.0; // Mount height
    private static final double FIELD_Z_COORDINATE = 0; // Assuming balls on field surface

    // Frame dimensions (OV5648 default for webcam, Husky Lens uses lower res)
    private int frameWidth = 640;
    private int frameHeight = 480;

    public DualCamera(Robot robot) {
        super(robot);
        currentDetections = new CopyOnWriteArrayList<>();
        detectedBallTargets = new CopyOnWriteArrayList<>();
        cameraOffsetConfig = CameraOffsetConfig.createCenterMounted();
    }

    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        // Initialize AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Initialize ball detection - choose between HSV or Husky Lens
        if (useHuskyLens) {
            // Initialize Husky Lens detector
            huskyLensBallDetector = new HuskyLensBallDetector(hardwareMap, "Husky Lens");
            ballDetectionProcessor = null;
        } else {
            // Initialize HSV ball detection processor
            ballDetectionProcessor = new BallDetectionProcessor(telemetry);
            huskyLensBallDetector = null;
        }

        // Create AprilTag Vision Portal
        VisionPortal.Builder aprilTagBuilder = new VisionPortal.Builder();
        aprilTagBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagBuilder.addProcessor(aprilTagProcessor);
        aprilTagPortal = aprilTagBuilder.build();

        // Create Ball Detection Vision Portal (only if using HSV, not Husky Lens)
        if (!useHuskyLens && ballDetectionProcessor != null) {
            VisionPortal.Builder ballDetectionBuilder = new VisionPortal.Builder();
            ballDetectionBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            ballDetectionBuilder.addProcessor(ballDetectionProcessor);
            ballDetectionPortal = ballDetectionBuilder.build();
        }
    }

    @Override
    public void startup() {
        super.startup();
        // Ensure both portals are streaming
        aprilTagPortal.resumeStreaming();
        if (ballDetectionPortal != null) {
            ballDetectionPortal.resumeStreaming();
        }
        // Initialize Husky Lens if using it
        if (useHuskyLens && huskyLensBallDetector != null) {
            huskyLensBallDetector.setEnabled(true);
        }
    }

    @Override
    public void shutdown() {
        super.shutdown();
        if (aprilTagPortal != null) {
            aprilTagPortal.close();
        }
        if (ballDetectionPortal != null) {
            ballDetectionPortal.close();
        }
    }

    @Override
    public void update(OpMode opMode) {
        super.update(opMode);

        // Update AprilTag detections
        currentDetections = aprilTagProcessor.getDetections();

        // Update ball detections
        if (useHuskyLens && huskyLensBallDetector != null) {
            huskyLensBallDetector.update();
            updateBallDetectionsFromHuskyLens();
        } else if (ballDetectionProcessor != null) {
            updateBallDetectionsFromHSV();
        }

        // Alternate between portals to reduce CPU load if needed
        if (robot.cycle % 3 == 0 && ballDetectionPortal != null) {
            aprilTagPortal.resumeStreaming();
            ballDetectionPortal.pauseStreaming();
        } else if (robot.cycle % 3 == 1 && ballDetectionPortal != null) {
            ballDetectionPortal.resumeStreaming();
            aprilTagPortal.pauseStreaming();
        }
    }

    /**
     * Update ball detections from Husky Lens
     */
    private void updateBallDetectionsFromHuskyLens() {
        detectedBallTargets.clear();

        if (huskyLensBallDetector == null) {
            return;
        }

        List<BallDetectionProcessor.BallDetection> ballDetections = huskyLensBallDetector.getDetections();
        Pose robotPose = robot.getRobotPose();

        for (BallDetectionProcessor.BallDetection ballDet : ballDetections) {
            // Convert pixel coordinates to field coordinates using offset config
            BallTarget fieldBall = cameraOffsetConfig.pixelToFieldCoordinates(
                    ballDet.x, ballDet.y, robotPose
            );
            if (fieldBall != null) {
                detectedBallTargets.add(fieldBall);
            }
        }
    }

    /**
     * Update ball detections from HSV-based webcam detection
     */
    private void updateBallDetectionsFromHSV() {
        detectedBallTargets.clear();

        if (ballDetectionProcessor == null) {
            return;
        }

        List<BallDetectionProcessor.BallDetection> ballDetections = ballDetectionProcessor.getDetectedBalls();
        Pose robotPose = robot.getRobotPose();

        for (BallDetectionProcessor.BallDetection ballDet : ballDetections) {
            // Convert pixel coordinates to field coordinates using offset config
            BallTarget fieldBall = cameraOffsetConfig.pixelToFieldCoordinates(
                    ballDet.x, ballDet.y, robotPose
            );
            if (fieldBall != null) {
                detectedBallTargets.add(fieldBall);
            }
        }
    }

    /**
     * Legacy method - now uses CameraOffsetConfig for conversion
     * Kept for backward compatibility
     */
    @Deprecated
    private BallTarget pixelToFieldCoordinates(double pixelX, double pixelY, double confidence) {
        // Use default center-mounted configuration
        Pose robotPose = robot.getRobotPose();
        if (robotPose == null) {
            return new BallTarget(pixelX, pixelY, confidence);
        }
        return cameraOffsetConfig.pixelToFieldCoordinates(pixelX, pixelY, robotPose);
    }

    /**
     * Get the closest detected ball
     */
    public BallTarget getClosestBall() {
        if (detectedBallTargets.isEmpty()) {
            return null;
        }

        Pose robotPose = robot.getRobotPose();
        if (robotPose == null) {
            // If no pose available, return the ball with highest confidence
            BallTarget best = detectedBallTargets.get(0);
            for (BallTarget ball : detectedBallTargets) {
                if (ball.confidence > best.confidence) {
                    best = ball;
                }
            }
            return best;
        }

        // Return ball with minimum distance
        BallTarget closest = detectedBallTargets.get(0);
        double minDist = closest.distanceTo(robotPose);

        for (BallTarget ball : detectedBallTargets) {
            double dist = ball.distanceTo(robotPose);
            if (dist < minDist) {
                minDist = dist;
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Get all detected ball targets in field coordinates
     */
    public List<BallTarget> getDetectedBalls() {
        return new ArrayList<>(detectedBallTargets);
    }

    /**
     * Get number of detected balls
     */
    public int getBallCount() {
        return detectedBallTargets.size();
    }

    /**
     * Get current robot pose from AprilTag detection
     */
    public Pose getPose() {
        if (robot.isRed()) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.metadata.name.contains("Red")) {
                    Pose3D robotPose3D = detection.robotPose;
                    double x = robotPose3D.getPosition().x;
                    double y = robotPose3D.getPosition().y;
                    double heading = robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                    return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                            .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                }
            }
        } else {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.metadata.name.contains("Blue")) {
                    Pose3D robotPose3D = detection.robotPose;
                    double x = robotPose3D.getPosition().x;
                    double y = robotPose3D.getPosition().y;
                    double heading = robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

                    return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                            .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                }
            }
        }
        return null;
    }

    /**
     * Set HSV color range for ball detection (only for HSV mode)
     */
    public void setBallColorRange(int hMin, int hMax, int sMin, int sMax, int vMin, int vMax) {
        if (ballDetectionProcessor != null) {
            ballDetectionProcessor.setColorRange(hMin, hMax, sMin, sMax, vMin, vMax);
        }
    }

    /**
     * Set minimum ball area threshold (only for HSV mode)
     */
    public void setMinBallArea(double minArea) {
        if (ballDetectionProcessor != null) {
            ballDetectionProcessor.setMinBallArea(minArea);
        }
    }

    /**
     * Enable Husky Lens for ball detection instead of HSV
     */
    public void useHuskyLensDetection() {
        this.useHuskyLens = true;
    }

    /**
     * Use HSV-based webcam detection for balls
     */
    public void useHSVDetection() {
        this.useHuskyLens = false;
    }

    /**
     * Set camera offset configuration for side-mounted cameras
     */
    public void setCameraOffsetConfig(CameraOffsetConfig config) {
        this.cameraOffsetConfig = config;
    }

    /**
     * Configure camera position relative to robot center
     */
    public void setCameraPosition(double offsetX, double offsetY, double offsetZ) {
        cameraOffsetConfig.setCameraPosition(offsetX, offsetY, offsetZ);
    }

    /**
     * Configure camera orientation
     */
    public void setCameraOrientation(double yaw, double pitch, double roll) {
        cameraOffsetConfig.setCameraOrientation(yaw, pitch, roll);
    }

    /**
     * Use preset: camera mounted on left side
     */
    public void useCameraLeftMounted(double distance) {
        cameraOffsetConfig = CameraOffsetConfig.createLeftMounted(distance);
    }

    /**
     * Use preset: camera mounted on right side
     */
    public void useCameraRightMounted(double distance) {
        cameraOffsetConfig = CameraOffsetConfig.createRightMounted(distance);
    }

    /**
     * Use preset: camera mounted forward
     */
    public void useCameraForwardMounted(double distance) {
        cameraOffsetConfig = CameraOffsetConfig.createForwardMounted(distance);
    }

    /**
     * Use preset: camera mounted at center
     */
    public void useCameraCenterMounted() {
        cameraOffsetConfig = CameraOffsetConfig.createCenterMounted();
    }

    /**
     * Check if AprilTag detection is available
     */
    public boolean hasAprilTagLock() {
        return !currentDetections.isEmpty();
    }

    /**
     * Check if any balls are detected
     */
    public boolean hasBallDetections() {
        return !detectedBallTargets.isEmpty();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);

        addLine("\n=== AprilTag Detection ===");
        addData("Tags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                addLine(String.format("\n[ID %d] %s", detection.id, detection.metadata.name));
                if (!detection.metadata.name.contains("Obelisk")) {
                    addLine(String.format("  XYZ: %.1f, %.1f, %.1f (in)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                }
            }
        }

        addLine("\n=== Ball Detection ===");
        addData("Balls Detected", detectedBallTargets.size());

        for (int i = 0; i < detectedBallTargets.size(); i++) {
            BallTarget ball = detectedBallTargets.get(i);
            addLine(String.format("  Ball %d: (%.1f, %.1f) conf=%.2f",
                    i + 1, ball.fieldX, ball.fieldY, ball.confidence));
        }

        if (!detectedBallTargets.isEmpty()) {
            BallTarget closest = getClosestBall();
            addLine(String.format("  Closest: (%.1f, %.1f)", closest.fieldX, closest.fieldY));
        }
    }
}
