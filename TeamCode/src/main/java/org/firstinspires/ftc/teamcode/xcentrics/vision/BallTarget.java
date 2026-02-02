package org.firstinspires.ftc.teamcode.xcentrics.vision;

import com.pedropathing.geometry.Pose;

/**
 * BallTarget - Represents a detected ball with its position in field coordinates
 * Converts pixel coordinates to field coordinates using camera calibration
 */
public class BallTarget {
    public double fieldX;      // Field X coordinate (inches)
    public double fieldY;      // Field Y coordinate (inches)
    public double confidence;  // Confidence of detection 0-1
    public long timestamp;     // When this detection was made

    public BallTarget(double fieldX, double fieldY, double confidence) {
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.confidence = confidence;
        this.timestamp = System.currentTimeMillis();
    }

    /**
     * Convert to Pedro Pose for pathfinding
     */
    public Pose toPose() {
        return new Pose(fieldX, fieldY);
    }

    /**
     * Get distance from given pose
     */
    public double distanceTo(Pose robotPose) {
        double dx = fieldX - robotPose.getX();
        double dy = fieldY - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public String toString() {
        return String.format("BallTarget(%.2f, %.2f, conf=%.2f)", fieldX, fieldY, confidence);
    }
}
