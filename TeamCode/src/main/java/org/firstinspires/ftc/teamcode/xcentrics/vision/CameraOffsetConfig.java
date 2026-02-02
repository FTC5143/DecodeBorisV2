package org.firstinspires.ftc.teamcode.xcentrics.vision;

import com.pedropathing.geometry.Pose;

/**
 * CameraOffsetConfig - Handles camera mounting position and offset compensation
 * 
 * Allows camera to be mounted off-center (left, right, forward, backward)
 * while maintaining accurate ball detection in field coordinates.
 */
public class CameraOffsetConfig {
    // Camera position relative to robot center
    private double offsetX = 0.0;      // Right offset (inches, positive = right)
    private double offsetY = 0.0;      // Forward offset (inches, positive = forward)
    private double offsetZ = 12.0;     // Height offset (inches)
    
    // Camera orientation
    private double yaw = 0.0;          // Heading offset (radians)
    private double pitch = -Math.PI/2; // Pitch down (radians, -90 degrees)
    private double roll = 0.0;         // Roll (radians)

    // Intrinsic camera parameters
    private double focalLengthX = 578.272;
    private double focalLengthY = 578.272;
    private double principalPointX = 402.145;
    private double principalPointY = 221.506;
    private int imageWidth = 320;
    private int imageHeight = 240;

    /**
     * Constructor with default mounting (center, 12 inches high, pointing down)
     */
    public CameraOffsetConfig() {
        // Defaults already set above
    }

    /**
     * Set camera position relative to robot center
     * 
     * @param offsetX Right offset in inches (positive = right)
     * @param offsetY Forward offset in inches (positive = forward)
     * @param offsetZ Height offset in inches above ground
     */
    public void setCameraPosition(double offsetX, double offsetY, double offsetZ) {
        this.offsetX = offsetX;
        this.offsetY = offsetY;
        this.offsetZ = offsetZ;
    }

    /**
     * Set camera orientation
     * 
     * @param yaw Camera heading (radians)
     * @param pitch Camera pitch, typically -PI/2 for downward pointing
     * @param roll Camera roll (radians)
     */
    public void setCameraOrientation(double yaw, double pitch, double roll) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    /**
     * Set camera intrinsic parameters
     */
    public void setCameraIntrinsics(double fx, double fy, double cx, double cy) {
        this.focalLengthX = fx;
        this.focalLengthY = fy;
        this.principalPointX = cx;
        this.principalPointY = cy;
    }

    /**
     * Set image resolution
     */
    public void setImageResolution(int width, int height) {
        this.imageWidth = width;
        this.imageHeight = height;
    }

    /**
     * Convert pixel coordinates to field coordinates accounting for camera offset
     * 
     * @param pixelX X coordinate in image (pixels)
     * @param pixelY Y coordinate in image (pixels)
     * @param robotPose Current robot pose in field
     * @return Ball position in field coordinates
     */
    public BallTarget pixelToFieldCoordinates(double pixelX, double pixelY, Pose robotPose) {
        // Step 1: Normalize pixel coordinates to camera frame
        double normalizedX = (pixelX - principalPointX) / focalLengthX;
        double normalizedY = (pixelY - principalPointY) / focalLengthY;

        // Step 2: Calculate ground distance using camera height
        double groundDistance = offsetZ / Math.tan(Math.atan(normalizedY));
        
        // Clamp to reasonable range
        if (groundDistance < 1.0 || groundDistance > 200.0) {
            return null;
        }

        // Step 3: Calculate ball position relative to camera
        double ballX_camera = groundDistance * normalizedX;
        double ballY_camera = groundDistance;

        // Step 4: Apply camera orientation (if not pointing straight down)
        // For downward-pointing camera (pitch = -PI/2), this simplifies the math
        double ballX_relative = ballX_camera;
        double ballY_relative = ballY_camera;

        // Step 5: Rotate by camera yaw and apply camera offset
        double cosYaw = Math.cos(yaw);
        double sinYaw = Math.sin(yaw);

        // Apply yaw rotation
        double ballX_rotated = ballX_relative * cosYaw - ballY_relative * sinYaw;
        double ballY_rotated = ballX_relative * sinYaw + ballY_relative * cosYaw;

        // Apply camera offset to get position relative to robot center
        double ballX_robot = ballX_rotated + offsetX;
        double ballY_robot = ballY_rotated + offsetY;

        // Step 6: Rotate by robot heading and translate to field coordinates
        double robotHeading = robotPose.getHeading();
        double cosRobotHeading = Math.cos(robotHeading);
        double sinRobotHeading = Math.sin(robotHeading);

        double fieldX = robotPose.getX() + 
                       (ballX_robot * cosRobotHeading - ballY_robot * sinRobotHeading);
        double fieldY = robotPose.getY() + 
                       (ballX_robot * sinRobotHeading + ballY_robot * cosRobotHeading);

        // Step 7: Create ball target
        double confidence = Math.min(1.0, 24.0 / groundDistance); // Confidence decreases with distance
        return new BallTarget(fieldX, fieldY, confidence);
    }

    // Getters
    public double getOffsetX() { return offsetX; }
    public double getOffsetY() { return offsetY; }
    public double getOffsetZ() { return offsetZ; }
    public double getYaw() { return yaw; }
    public double getPitch() { return pitch; }
    public double getRoll() { return roll; }
    public double getFocalLengthX() { return focalLengthX; }
    public double getFocalLengthY() { return focalLengthY; }
    public double getPrincipalPointX() { return principalPointX; }
    public double getPrincipalPointY() { return principalPointY; }
    public int getImageWidth() { return imageWidth; }
    public int getImageHeight() { return imageHeight; }

    /**
     * Get string representation for debugging
     */
    @Override
    public String toString() {
        return String.format("CameraOffset(X:%.1f Y:%.1f Z:%.1f, Yaw:%.2f)",
                offsetX, offsetY, offsetZ, Math.toDegrees(yaw));
    }

    /**
     * Preset: Center mounted camera
     */
    public static CameraOffsetConfig createCenterMounted() {
        CameraOffsetConfig config = new CameraOffsetConfig();
        config.setCameraPosition(0, 0, 12.0);
        return config;
    }

    /**
     * Preset: Left-mounted camera (6 inches left of center)
     */
    public static CameraOffsetConfig createLeftMounted(double distanceLeft) {
        CameraOffsetConfig config = new CameraOffsetConfig();
        config.setCameraPosition(-distanceLeft, 0, 12.0);
        return config;
    }

    /**
     * Preset: Right-mounted camera (6 inches right of center)
     */
    public static CameraOffsetConfig createRightMounted(double distanceRight) {
        CameraOffsetConfig config = new CameraOffsetConfig();
        config.setCameraPosition(distanceRight, 0, 12.0);
        return config;
    }

    /**
     * Preset: Forward-mounted camera (forward of center)
     */
    public static CameraOffsetConfig createForwardMounted(double distanceForward) {
        CameraOffsetConfig config = new CameraOffsetConfig();
        config.setCameraPosition(0, distanceForward, 12.0);
        return config;
    }
}
