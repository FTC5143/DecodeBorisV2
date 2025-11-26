package org.firstinspires.ftc.teamcode.xcentrics.util.math;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Objects;

/**
 * A utility class for calculating the optimal launch angle (hood angle) and initial velocity
 * for a projectile shooter (e.g., in FTC or FRC). This uses the minimum-velocity trajectory
 * to hit a target at horizontal distance x and height difference y, assuming no air resistance
 * and constant gravity g.
 *
 * Additionally, provides a method to set the flywheel motor velocity on a DcMotorEx object
 * based on the calculated initial velocity v₀, converting to RPM using the flywheel radius.
 *
 * The formulas are derived from projectile motion equations:
 * - Straight-line distance r = sqrt(x² + y²)
 * - Base angle φ = arctan(y / x)
 * - Optimal angle θ = 45° + (1/2)φ (in degrees)
 * - Minimal velocity v₀ = sqrt(g * r * (1 + y / r))
 *
 * For RPM: angular_velocity (rad/s) = v₀ / radius; RPM = (angular_velocity * 60) / (2π)
 * Assumes direct drive or geared setup where motor RPM ≈ flywheel RPM; adjust gearing factor if needed.
 *
 * Usage: Call calculateOptimalTrajectory(x, y, g) to get results, then setFlywheelVelocity(motor, result.getInitialVelocityMs(), radius).
 */
public class ShooterTrajectoryCalculator {

    /**
     * Private constructor to prevent instantiation (utility class).
     */
    private ShooterTrajectoryCalculator() {
        throw new UnsupportedOperationException("Utility class - do not instantiate.");
    }

    /**
     * Calculates the optimal launch angle (in degrees) and initial velocity (m/s)
     * for hitting a target at horizontal distance x and vertical height y.
     *
     * @param x Horizontal distance to target (m), must be > 0.
     * @param y Vertical height difference (target - launch height, m), typically > 0.
     * @param g Acceleration due to gravity (m/s²), default ~9.81.
     * @return A TrajectoryResult object containing θ (degrees) and v0 (m/s).
     * @throws IllegalArgumentException if x <= 0 or inputs are invalid.
     */
    public static TrajectoryResult calculateOptimalTrajectory(double x, double y, double g) {
        Objects.requireNonNull(Double.valueOf(x), "x must not be null");
        Objects.requireNonNull(Double.valueOf(y), "y must not be null");
        Objects.requireNonNull(Double.valueOf(g), "g must not be null");

        if (x <= 0) {
            throw new IllegalArgumentException("Horizontal distance x must be positive (> 0 m)");
        }
        if (g <= 0) {
            throw new IllegalArgumentException("Gravity g must be positive (> 0 m/s²)");
        }

        // Step 1: Compute straight-line distance r
        double r = Math.sqrt(x * x + y * y);

        // Step 2: Compute base angle φ in radians, then convert to degrees
        double phiRad = Math.atan(y / x);
        double phiDeg = Math.toDegrees(phiRad);

        // Step 3: Optimal launch angle θ in degrees
        double thetaDeg = 45.0 + 0.5 * phiDeg;

        // Step 4: Minimal initial velocity v₀
        double v0 = Math.sqrt(g * r * (1.0 + y / r));

        return new TrajectoryResult(thetaDeg, v0);
    }

    /**
     * Sets the velocity (in RPM) on the DcMotorEx flywheel motor to achieve the target ball exit velocity v₀.
     * This assumes a single flywheel or effective tangential speed; for dual flywheels, the velocity is similar.
     *
     * @param motor The DcMotorEx object controlling the GoBilda 6000 RPM motor.
     * @param v0 The desired initial ball velocity (m/s) from trajectory calculation.
     * @param flywheelRadius The radius of the flywheel (m), e.g., 0.06 for a 120mm diameter wheel.
     * @throws IllegalArgumentException if inputs are invalid.
     */
    public static void setFlywheelVelocity(DcMotorEx motor, double v0, double flywheelRadius) {
        Objects.requireNonNull(motor, "Motor must not be null");
        Objects.requireNonNull(Double.valueOf(v0), "v0 must not be null");
        Objects.requireNonNull(Double.valueOf(flywheelRadius), "flywheelRadius must not be null");

        if (v0 < 0) {
            throw new IllegalArgumentException("Initial velocity v0 must be non-negative (>= 0 m/s)");
        }
        if (flywheelRadius <= 0) {
            throw new IllegalArgumentException("Flywheel radius must be positive (> 0 m)");
        }

        // Convert v₀ (m/s) to angular velocity (rad/s)
        double angularVelocityRadS = v0 / flywheelRadius;

        // Convert to RPM
        double targetRPM = (angularVelocityRadS * 60.0) / (2.0 * Math.PI);

        // Set the motor velocity (assumes velocity PID is configured in hardware config)
        motor.setVelocity(targetRPM);
    }

    /**
     * A simple immutable class to hold the results (compatible with Java 8+).
     */
    public static class TrajectoryResult {
        private final double launchAngleDegrees;
        private final double initialVelocityMs;

        public TrajectoryResult(double launchAngleDegrees, double initialVelocityMs) {
            this.launchAngleDegrees = launchAngleDegrees;
            this.initialVelocityMs = initialVelocityMs;
        }

        public double getLaunchAngleDegrees() {
            return launchAngleDegrees;
        }

        public double getInitialVelocityMs() {
            return initialVelocityMs;
        }

        @Override
        public String toString() {
            return String.format("θ = %.2f° | v₀ = %.2f m/s", launchAngleDegrees, initialVelocityMs);
        }
    }

    /**
     * Example usage method for testing (optional - can be removed). Note: Requires a real DcMotorEx in a full FTC OpMode.
     */
    public static void main(String[] args) {
        // Example: x=2m, y=1m, g=9.81 m/s²
        TrajectoryResult result = calculateOptimalTrajectory(2.0, 1.0, 9.81);
        System.out.println("Optimal Trajectory: " + result);
        // Output: Optimal Trajectory: θ = 58.28° | v₀ = 5.63 m/s

        // Example: Set motor (pseudo - needs actual motor object)
        // DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        // setFlywheelVelocity(flywheelMotor, result.getInitialVelocityMs(), 0.06); // 60mm radius
    }
}