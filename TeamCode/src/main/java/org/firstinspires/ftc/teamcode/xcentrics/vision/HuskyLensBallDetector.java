package org.firstinspires.ftc.teamcode.xcentrics.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.ArrayList;
import java.util.List;

/**
 * HuskyLensBallDetector - Communicates with Husky Lens camera for real-time ball detection
 * 
 * Features:
 * - Direct I2C communication with Husky Lens
 * - Real-time ball detection results
 * - Algorithm selection (color recognition, line tracking, etc.)
 * - Confidence-based filtering
 * 
 * Note: Husky Lens must be pre-configured with ball detection algorithm
 */
public class HuskyLensBallDetector {
    private HuskyLens huskyLens;
    private List<BallDetectionProcessor.BallDetection> detections;
    private int maxObjectsToTrack = 5;
    private double minConfidence = 0.0;

    /**
     * Initialize Husky Lens for ball detection
     */
    public HuskyLensBallDetector(HardwareMap hardwareMap, String deviceName) {
        huskyLens = hardwareMap.get(HuskyLens.class, deviceName);
        detections = new ArrayList<>();
        
        if (huskyLens != null) {
            huskyLens.knock();
            if (huskyLens.isBeeping()) {
                huskyLens.stop();
            }
        }
    }

    /**
     * Update detections from Husky Lens
     * Should be called regularly in the main loop
     */
    public void update() {
        if (huskyLens == null) {
            return;
        }

        detections.clear();

        // Request all blocks (objects) detected by Husky Lens
        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks != null && blocks.length > 0) {
            for (int i = 0; i < Math.min(blocks.length, maxObjectsToTrack); i++) {
                HuskyLens.Block block = blocks[i];

                if (block.width > 0 && block.height > 0) {
                    BallDetectionProcessor.BallDetection detection = new BallDetectionProcessor.BallDetection();

                    // Convert Husky Lens coordinates to our format
                    detection.x = block.x;
                    detection.y = block.y;
                    detection.width = block.width;
                    detection.height = block.height;
                    detection.area = block.width * block.height;
                    
                    // Confidence based on size and detection quality
                    detection.confidence = Math.min(1.0, (double) block.width / 320.0);

                    // Apply confidence filter
                    if (detection.confidence >= minConfidence) {
                        detections.add(detection);
                    }
                }
            }
        }
    }

    /**
     * Get all detected balls
     */
    public List<BallDetectionProcessor.BallDetection> getDetections() {
        return new ArrayList<>(detections);
    }

    /**
     * Get number of detected balls
     */
    public int getDetectionCount() {
        return detections.size();
    }

    /**
     * Get closest ball (by size, which indicates proximity)
     */
    public BallDetectionProcessor.BallDetection getClosestBall() {
        if (detections.isEmpty()) {
            return null;
        }

        // Largest detection (closest to camera)
        BallDetectionProcessor.BallDetection closest = detections.get(0);
        for (BallDetectionProcessor.BallDetection det : detections) {
            if (det.area > closest.area) {
                closest = det;
            }
        }
        return closest;
    }

    /**
     * Set minimum confidence threshold for detections
     */
    public void setMinConfidence(double confidence) {
        this.minConfidence = confidence;
    }

    /**
     * Set maximum number of objects to track
     */
    public void setMaxObjectsToTrack(int max) {
        this.maxObjectsToTrack = max;
    }

    /**
     * Select algorithm on Husky Lens
     * Common algorithms: COLOR_RECOGNITION, LINE_TRACKING, OBJECT_CLASSIFICATION
     */
    public void selectAlgorithm(int algorithmId) {
        if (huskyLens != null) {
            huskyLens.selectAlgorithm(algorithmId);
        }
    }

    /**
     * Enable or disable Husky Lens
     */
    public void setEnabled(boolean enabled) {
        if (huskyLens != null && enabled) {
            huskyLens.knock();
        }
    }

    /**
     * Check if Husky Lens is connected and responding
     */
    public boolean isConnected() {
        return huskyLens != null && huskyLens.isBeeping();
    }
}
