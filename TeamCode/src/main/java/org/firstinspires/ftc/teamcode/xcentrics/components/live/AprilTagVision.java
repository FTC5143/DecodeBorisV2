package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Enhanced AprilTag Vision System with Pose Fusion
 * Uses AprilTag detection for real-time field localization
 * Maintains robust position estimate even when tags are partially occluded
 * @author 5143 Xcentrics
 */
@Configurable
class AprilTagVisionConfig {
    public static double confidenceThreshold = 0.5;
    public static double maxPoseUpdateDistance = 100.0; // max inches before rejecting pose update
    public static int poseFusionHistorySize = 5; // number of poses to average
    public static boolean enablePoseFusion = true;
    public static double fusionWeight = 0.7; // 70% vision, 30% odometry
}

public class AprilTagVision extends Component {
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public List<AprilTagDetection> currentDetections;
    
    private Pose lastEstimatedPose = null;
    private List<Pose> poseHistory = new java.util.LinkedList<>();
    private long lastUpdateTime = 0;
    private boolean isInitialized = false;
    
    // Tracking which tags are visible
    private boolean redTagVisible = false;
    private boolean blueTagVisible = false;
    private AprilTagDetection lastValidRedTag = null;
    private AprilTagDetection lastValidBlueTag = null;
    
    {
        name = "AprilTag Vision";
    }
    
    /**
     * Represents detected AprilTag data with timestamp
     */
    public static class TagDetection {
        public int id;
        public Pose robotPose;
        public double distance;
        public long detectionTime;
        public String alliance; // "RED" or "BLUE"
        
        public TagDetection(int id, Pose robotPose, double distance, String alliance) {
            this.id = id;
            this.robotPose = robotPose;
            this.distance = distance;
            this.alliance = alliance;
            this.detectionTime = System.currentTimeMillis();
        }
    }
    
    public AprilTagVision(Robot robot) {
        super(robot);
        cameraPosition = new Position(DistanceUnit.MM,
                0, 0, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90, 0, 0);
    }
    
    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        super.registerHardware(hardwareMap);
        
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .build();
            
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.addProcessor(aprilTag);
            visionPortal = builder.build();
            
            isInitialized = true;
            status = STATUS_ONLINE;
        } catch (Exception e) {
            addLine("ERROR: Could not initialize AprilTag Vision: " + e.getMessage());
            status = STATUS_OFFLINE;
        }
    }
    
    @Override
    public void startup() {
        super.startup();
        if (isInitialized && visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }
    
    @Override
    public void update(LinearOpMode opMode) {
        super.update(opMode);
        
        if (!isInitialized || aprilTag == null) {
            return;
        }
        
        try {
            currentDetections = aprilTag.getDetections();
            redTagVisible = false;
            blueTagVisible = false;
            
            // Process detections
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.metadata.name.contains("Red")) {
                        redTagVisible = true;
                        lastValidRedTag = detection;
                    } else if (detection.metadata.name.contains("Blue")) {
                        blueTagVisible = true;
                        lastValidBlueTag = detection;
                    }
                }
            }
            
            // Update pose based on current alliance
            updatePoseEstimate();
            lastUpdateTime = System.currentTimeMillis();
            status = STATUS_ONLINE;
            
        } catch (Exception e) {
            addLine("ERROR: AprilTag update failed: " + e.getMessage());
            status = STATUS_OFFLINE;
        }
    }
    
    /**
     * Update pose estimate with pose fusion
     */
    private void updatePoseEstimate() {
        AprilTagDetection detection = null;
        
        // Select appropriate tag based on alliance
        if (robot.isRed() && lastValidRedTag != null) {
            detection = lastValidRedTag;
        } else if (!robot.isRed() && lastValidBlueTag != null) {
            detection = lastValidBlueTag;
        }
        
        if (detection != null) {
            Pose visionPose = extractPose(detection);
            
            if (visionPose != null) {
                // Perform pose fusion if enabled
                if (AprilTagVisionConfig.enablePoseFusion && lastEstimatedPose != null) {
                    // Check distance validity
                    double poseDistance = lastEstimatedPose.distanceFrom(visionPose);
                    if (poseDistance < AprilTagVisionConfig.maxPoseUpdateDistance) {
                        // Fuse poses with weighting
                        visionPose = fusePoses(lastEstimatedPose, visionPose);
                    }
                }
                
                lastEstimatedPose = visionPose;
                addPoseToHistory(visionPose);
            }
        }
    }
    
    /**
     * Extract pose from AprilTag detection
     */
    private Pose extractPose(AprilTagDetection detection) {
        if (detection == null) {
            return null;
        }
        
        try {
            Pose3D robotPose3D = detection.robotPose;
            double x = robotPose3D.getPosition().x;
            double y = robotPose3D.getPosition().y;
            double heading = robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS);
            
            return new Pose(x, y, heading, FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        } catch (Exception e) {
            return null;
        }
    }
    
    /**
     * Fuse two pose estimates with weighted averaging
     */
    private Pose fusePoses(Pose odometryPose, Pose visionPose) {
        double weight = AprilTagVisionConfig.fusionWeight;
        
        double x = (visionPose.getX() * weight) + (odometryPose.getX() * (1 - weight));
        double y = (visionPose.getY() * weight) + (odometryPose.getY() * (1 - weight));
        
        // Handle heading fusion with angle wrapping
        double visionHeading = visionPose.getHeading();
        double odometryHeading = odometryPose.getHeading();
        
        // Wrap heading difference to [-pi, pi]
        double headingDiff = Math.atan2(
                Math.sin(visionHeading - odometryHeading),
                Math.cos(visionHeading - odometryHeading)
        );
        
        double heading = odometryHeading + (headingDiff * weight);
        
        return new Pose(x, y, heading);
    }
    
    /**
     * Maintain history of poses for stability
     */
    private void addPoseToHistory(Pose pose) {
        if (poseHistory.size() >= AprilTagVisionConfig.poseFusionHistorySize) {
            poseHistory.remove(0);
        }
        poseHistory.add(pose);
    }
    
    @Override
    @SuppressLint("DefaultLocale")
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        
        if (!isInitialized) {
            addLine("AprilTag Vision: NOT INITIALIZED");
            return;
        }
        
        addData("AprilTags Detected", currentDetections.size());
        addData("Red Tag Visible", redTagVisible);
        addData("Blue Tag Visible", blueTagVisible);
        
        if (lastEstimatedPose != null) {
            addData("Pose X", String.format("%.2f", lastEstimatedPose.getX()));
            addData("Pose Y", String.format("%.2f", lastEstimatedPose.getY()));
            addData("Pose Heading", String.format("%.2f°", Math.toDegrees(lastEstimatedPose.getHeading())));
        }
        
        addData("Pose History Size", poseHistory.size());
        addData("Time Since Update", (System.currentTimeMillis() - lastUpdateTime) + "ms");
    }
    
    @Override
    public void shutdown() {
        super.shutdown();
        if (visionPortal != null) {
            visionPortal.stopStreaming();
            visionPortal.close();
        }
    }
    
    // ============== PUBLIC API ==============
    
    /**
     * Get current estimated robot pose from AprilTag vision
     * Fused with odometry if enabled
     */
    public Pose getPose() {
        return lastEstimatedPose;
    }
    
    /**
     * Get average pose from history (smoother estimate)
     */
    public Pose getAveragePose() {
        if (poseHistory.isEmpty()) {
            return lastEstimatedPose;
        }
        
        double sumX = 0, sumY = 0, sumCosH = 0, sumSinH = 0;
        for (Pose p : poseHistory) {
            sumX += p.getX();
            sumY += p.getY();
            sumCosH += Math.cos(p.getHeading());
            sumSinH += Math.sin(p.getHeading());
        }
        
        double avgX = sumX / poseHistory.size();
        double avgY = sumY / poseHistory.size();
        double avgHeading = Math.atan2(sumSinH / poseHistory.size(), sumCosH / poseHistory.size());
        
        return new Pose(avgX, avgY, avgHeading);
    }
    
    /**
     * Check if AprilTag detection is currently valid and fresh
     */
    public boolean isPoseValid() {
        if (lastEstimatedPose == null) {
            return false;
        }
        
        // Pose valid if updated within last 500ms
        return (System.currentTimeMillis() - lastUpdateTime) < 500;
    }
    
    /**
     * Get all current tag detections
     */
    public List<AprilTagDetection> getDetections() {
        return currentDetections;
    }
    
    /**
     * Check specific tag visibility
     */
    public boolean isRedTagVisible() {
        return redTagVisible;
    }
    
    public boolean isBlueTagVisible() {
        return blueTagVisible;
    }
    
    /**
     * Get time since last valid pose update (milliseconds)
     */
    public long getTimeSinceLastUpdate() {
        return System.currentTimeMillis() - lastUpdateTime;
    }
    
    /**
     * Set streaming state for performance optimization
     */
    public void setStreaming(boolean enabled) {
        if (visionPortal != null) {
            if (enabled) {
                visionPortal.resumeStreaming();
            } else {
                visionPortal.stopStreaming();
            }
        }
    }
}
