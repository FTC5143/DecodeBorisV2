package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Ball Detection using Husky Lens Vision Sensor
 * Continuously scans for balls and prioritizes by proximity
 * @author 5143 Xcentrics
 */
@Configurable
class BallDetectionConfig {
    public static double minBallConfidence = 0.5;
    public static double maxBallDistance = 500.0; // pixels from center
    public static int minBlockSize = 20; // minimum pixel area
    public static double pixelsPerInch = 8.0; // tune for your camera height/FOV
    public static double minForwardInches = 4.0; // clamp for stability
    public static double maxForwardInches = 72.0; // clamp for stability
}

public class BallDetectionVision extends Component {
    private HuskyLens huskyLens;
    private List<BallDetection> detectedBalls = new ArrayList<>();
    private BallDetection closestBall = null;
    private static final int FRAME_WIDTH = 320;
    private static final int FRAME_HEIGHT = 240;
    private static final double FRAME_CENTER_X = FRAME_WIDTH / 2.0;
    private long lastUpdateTime = 0;
    private boolean isConnected = false;
    
    /**
     * Represents a detected ball with position and confidence
     */
    public static class BallDetection {
        public int x;              // X position in frame (0-320)
        public int y;              // Y position in frame (0-240)
        public int width;          // Block width in pixels
        public int height;         // Block height in pixels
        public long id;            // Unique block ID
        public double confidence;  // Confidence level (0-1)
        public long detectionTime; // When this was detected
        
        public BallDetection(int x, int y, int width, int height, long id, double confidence) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.id = id;
            this.confidence = confidence;
            this.detectionTime = System.currentTimeMillis();
        }
        
        /**
         * Calculate distance from frame center
         */
        public double getDistanceFromCenter() {
            double centerX = FRAME_WIDTH / 2.0;
            double centerY = FRAME_HEIGHT / 2.0;
            return Math.sqrt(Math.pow(x - centerX, 2) + Math.pow(y - centerY, 2));
        }
        
        /**
         * Get ball area in pixels
         */
        public int getArea() {
            return width * height;
        }
        
        /**
         * Get position as angle relative to frame center
         * Positive angle = right, negative = left
         */
        public double getAngleFromCenter() {
            double centerX = FRAME_WIDTH / 2.0;
            return Math.atan2(x - centerX, centerY / 2.0); // Approximate based on frame geometry
        }
    }

    /**
     * Represents a detected ball transformed into robot/field coordinates
     */
    public static class BallObservation {
        public long id;
        public double forwardInches; // +X (forward)
        public double rightInches;   // +Y (right)
        public com.pedropathing.geometry.Pose fieldPose;
        public long detectionTime;

        public BallObservation(long id, double forwardInches, double rightInches,
                               com.pedropathing.geometry.Pose fieldPose, long detectionTime) {
            this.id = id;
            this.forwardInches = forwardInches;
            this.rightInches = rightInches;
            this.fieldPose = fieldPose;
            this.detectionTime = detectionTime;
        }
    }
    
    {
        name = "Ball Detection";
    }
    
    public BallDetectionVision(Robot robot) {
        super(robot);
    }
    
    @Override
    public void registerHardware(HardwareMap hardwareMap) {
        super.registerHardware(hardwareMap);
        try {
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            isConnected = true;
        } catch (Exception e) {
            isConnected = false;
            addLine("ERROR: Could not initialize HuskyLens: " + e.getMessage());
        }
    }
    
    @Override
    public void startup() {
        super.startup();
        if (isConnected) {
            // Attempt to knock/ping the device
            if (!huskyLens.knock()) {
                addLine("WARNING: HuskyLens not responding");
            }
        }
    }
    
    @Override
    public void update(LinearOpMode opMode) {
        super.update(opMode);
        
        if (!isConnected) {
            return;
        }
        
        try {
            // Get all detected blocks from HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks();
            detectedBalls.clear();
            
            if (blocks != null && blocks.length > 0) {
                for (HuskyLens.Block block : blocks) {
                    // Filter by size and position
                    if (block.width >= BallDetectionConfig.minBlockSize && 
                        block.height >= BallDetectionConfig.minBlockSize) {
                        
                        // Create ball detection
                        BallDetection ball = new BallDetection(
                            block.x, block.y, block.width, block.height,
                            block.id, 1.0 // HuskyLens confidence is implicit in detection
                        );
                        
                        // Filter by max distance from center
                        if (ball.getDistanceFromCenter() <= BallDetectionConfig.maxBallDistance) {
                            detectedBalls.add(ball);
                        }
                    }
                }
                
                // Sort by distance from center (prioritize centered balls first, then by area)
                detectedBalls.sort(Comparator
                    .comparingDouble(BallDetection::getDistanceFromCenter)
                    .thenComparingInt(BallDetection::getArea)
                    .reversed()
                );
                
                // Set closest ball as the primary target
                if (!detectedBalls.isEmpty()) {
                    closestBall = detectedBalls.get(0);
                }
            }
            
            lastUpdateTime = System.currentTimeMillis();
            status = STATUS_ONLINE;
            
        } catch (Exception e) {
            addLine("ERROR: HuskyLens update failed: " + e.getMessage());
            status = STATUS_OFFLINE;
        }
    }
    
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        
        if (!isConnected) {
            addLine("Ball Detection: DISCONNECTED");
            return;
        }
        
        addData("Balls Detected", detectedBalls.size());
        
        if (closestBall != null) {
            addData("Closest Ball X", closestBall.x);
            addData("Closest Ball Y", closestBall.y);
            addData("Closest Ball Size", closestBall.width + "x" + closestBall.height);
            addData("Distance from Center", String.format("%.1f", closestBall.getDistanceFromCenter()));
        } else {
            addLine("No balls detected");
        }
        
        if (detectedBalls.size() > 1) {
            addLine("Secondary balls available: " + (detectedBalls.size() - 1));
        }
    }
    
    // ============== PUBLIC API ==============
    
    /**
     * Get the closest/primary ball detection
     * @return BallDetection or null if no balls detected
     */
    public BallDetection getClosestBall() {
        return closestBall;
    }
    
    /**
     * Get all detected balls sorted by priority
     * @return List of BallDetection objects
     */
    public List<BallDetection> getAllDetectedBalls() {
        return new ArrayList<>(detectedBalls);
    }

    /**
     * Convert a ball detection into a robot/field-relative observation
     * Uses a simplified pixel-to-inch projection; tune BallDetectionConfig.pixelsPerInch.
     */
    public BallObservation toObservation(BallDetection detection, com.pedropathing.geometry.Pose robotPose) {
        if (detection == null || robotPose == null) {
            return null;
        }

        double rightInches = (detection.x - FRAME_CENTER_X) / BallDetectionConfig.pixelsPerInch;
        double forwardInches = (FRAME_HEIGHT - detection.y) / BallDetectionConfig.pixelsPerInch;

        forwardInches = Math.max(BallDetectionConfig.minForwardInches,
                Math.min(BallDetectionConfig.maxForwardInches, forwardInches));

        double heading = robotPose.getHeading();
        double fieldX = robotPose.getX() + forwardInches * Math.cos(heading) - rightInches * Math.sin(heading);
        double fieldY = robotPose.getY() + forwardInches * Math.sin(heading) + rightInches * Math.cos(heading);

        return new BallObservation(
                detection.id,
                forwardInches,
                rightInches,
                new com.pedropathing.geometry.Pose(fieldX, fieldY, heading),
                detection.detectionTime
        );
    }

    /**
     * Get observations for all detected balls in field coordinates
     */
    public List<BallObservation> getBallObservations(com.pedropathing.geometry.Pose robotPose) {
        List<BallObservation> observations = new ArrayList<>();
        if (robotPose == null) {
            return observations;
        }

        for (BallDetection detection : detectedBalls) {
            BallObservation obs = toObservation(detection, robotPose);
            if (obs != null) {
                observations.add(obs);
            }
        }

        return observations;
    }

    /**
     * Fill a provided list with observations to reduce allocations
     */
    public void getBallObservations(com.pedropathing.geometry.Pose robotPose, List<BallObservation> out) {
        out.clear();
        if (robotPose == null) {
            return;
        }

        for (BallDetection detection : detectedBalls) {
            BallObservation obs = toObservation(detection, robotPose);
            if (obs != null) {
                out.add(obs);
            }
        }
    }
    
    /**
     * Get ball count
     */
    public int getBallCount() {
        return detectedBalls.size();
    }
    
    /**
     * Check if balls are currently being detected
     */
    public boolean areBallsDetected() {
        return !detectedBalls.isEmpty() && isConnected;
    }
    
    /**
     * Get time since last update (milliseconds)
     */
    public long getTimeSinceLastUpdate() {
        return System.currentTimeMillis() - lastUpdateTime;
    }
    
    /**
     * Get connection status
     */
    public boolean isConnected() {
        return isConnected;
    }
}
