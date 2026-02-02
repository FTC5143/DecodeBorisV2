package org.firstinspires.ftc.teamcode.xcentrics.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * BallDetectionProcessor - Detects colored balls using HSV color space
 * Designed to work with the Husky Lens camera for real-time ball detection
 */
public class BallDetectionProcessor implements VisionProcessor {
    private Mat mat = new Mat();
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private List<BallDetection> detectedBalls = new ArrayList<>();
    private Telemetry telemetry;

    // HSV color range for ball detection (yellow/orange ball detection)
    private int hMin = 15;
    private int hMax = 40;
    private int sMin = 100;
    private int sMax = 255;
    private int vMin = 100;
    private int vMax = 255;

    // Minimum contour area to be considered a ball
    private double minBallArea = 100;

    public BallDetectionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialize processor
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeMs) {
        mat = frame.clone();

        // Convert BGR to HSV for better color detection
        Imgproc.cvtColor(mat, hsv, Imgproc.COLOR_BGR2HSV);

        // Create mask for the target color range
        org.opencv.core.Scalar lowerBound = new Scalar(hMin, sMin, vMin);
        org.opencv.core.Scalar upperBound = new Scalar(hMax, sMax, vMax);
        Core.inRange(hsv, lowerBound, upperBound, mask);

        // Find contours in the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedBalls.clear();

        // Process each contour
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area > minBallArea) {
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Create ball detection
                BallDetection detection = new BallDetection();
                detection.x = boundingRect.x + boundingRect.width / 2.0;
                detection.y = boundingRect.y + boundingRect.height / 2.0;
                detection.width = boundingRect.width;
                detection.height = boundingRect.height;
                detection.area = area;
                detection.confidence = Math.min(1.0, area / 5000.0); // Normalize area to confidence

                detectedBalls.add(detection);

                // Draw detection on frame
                Imgproc.rectangle(mat, boundingRect, new Scalar(0, 255, 0), 2);
                Imgproc.circle(mat, new org.opencv.core.Point(detection.x, detection.y), 5, new Scalar(255, 0, 0), -1);
            }
        }

        hierarchy.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Drawing is handled by processFrame
    }

    /**
     * Get the closest detected ball to the robot
     */
    public BallDetection getClosestBall() {
        if (detectedBalls.isEmpty()) {
            return null;
        }

        // Sort by area (larger = closer)
        BallDetection closest = detectedBalls.get(0);
        for (BallDetection ball : detectedBalls) {
            if (ball.area > closest.area) {
                closest = ball;
            }
        }
        return closest;
    }

    /**
     * Get all detected balls
     */
    public List<BallDetection> getDetectedBalls() {
        return new ArrayList<>(detectedBalls);
    }

    /**
     * Get the number of detected balls
     */
    public int getBallCount() {
        return detectedBalls.size();
    }

    /**
     * Set HSV color range for detection
     */
    public void setColorRange(int hMin, int hMax, int sMin, int sMax, int vMin, int vMax) {
        this.hMin = hMin;
        this.hMax = hMax;
        this.sMin = sMin;
        this.sMax = sMax;
        this.vMin = vMin;
        this.vMax = vMax;
    }

    /**
     * Set minimum ball area threshold
     */
    public void setMinBallArea(double minArea) {
        this.minBallArea = minArea;
    }

    /**
     * Data class for ball detection results
     */
    public static class BallDetection {
        public double x;           // Center X in pixels
        public double y;           // Center Y in pixels
        public double width;       // Width in pixels
        public double height;      // Height in pixels
        public double area;        // Area in pixels
        public double confidence;  // Confidence 0-1

        @Override
        public String toString() {
            return String.format("Ball(%.1f, %.1f, conf=%.2f)", x, y, confidence);
        }
    }
}
