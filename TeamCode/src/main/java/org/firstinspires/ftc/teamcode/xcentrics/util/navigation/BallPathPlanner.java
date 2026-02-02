package org.firstinspires.ftc.teamcode.xcentrics.util.navigation;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;

import java.util.ArrayList;
import java.util.List;

/**
 * BallPathPlanner - Advanced pathfinding for ball pickup sequences
 * 
 * Features:
 * - Multi-ball pickup planning
 * - Optimal ball ordering (nearest neighbor, distance minimization)
 * - Smooth curve generation for ball approaches
 * - Obstacle avoidance awareness
 * - Return path generation
 */
public class BallPathPlanner {
    private final Follower follower;
    private Pose startPose;
    private Pose scoringPose;
    private double approachDistance = 6.0; // inches before ball

    public BallPathPlanner(Follower follower) {
        this.follower = follower;
    }

    /**
     * Set the starting position for pathfinding
     */
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    /**
     * Set the final scoring position
     */
    public void setScoringPose(Pose scoringPose) {
        this.scoringPose = scoringPose;
    }

    /**
     * Set approach distance (how close to get before pickup)
     */
    public void setApproachDistance(double distance) {
        this.approachDistance = distance;
    }

    /**
     * Create a path to a single ball target
     */
    public PathChain createPathToBall(Pose currentPose, BallTarget target) {
        Pose ballPose = target.toPose();

        // Calculate approach point (stop slightly before ball)
        double angle = Math.atan2(
                ballPose.getY() - currentPose.getY(),
                ballPose.getX() - currentPose.getX()
        );

        Pose approachPose = new Pose(
                ballPose.getX() - approachDistance * Math.cos(angle),
                ballPose.getY() - approachDistance * Math.sin(angle),
                angle
        );

        // Create smooth bezier path
        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, approachPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), angle)
                .build();
    }

    /**
     * Create an optimized path through multiple balls using nearest neighbor algorithm
     */
    public List<PathChain> createMultiBallPath(Pose currentPose, List<BallTarget> balls) {
        List<PathChain> paths = new ArrayList<>();

        if (balls.isEmpty()) {
            return paths;
        }

        // Sort balls using nearest neighbor
        List<BallTarget> sortedBalls = nearestNeighborSort(currentPose, balls);

        Pose lastPose = currentPose;
        for (BallTarget ball : sortedBalls) {
            PathChain pathToBall = createPathToBall(lastPose, ball);
            paths.add(pathToBall);
            lastPose = ball.toPose();
        }

        return paths;
    }

    /**
     * Create a return path to scoring position
     */
    public PathChain createReturnPath(Pose currentPose) {
        if (scoringPose == null) {
            scoringPose = startPose;
        }

        return follower.pathBuilder()
                .addPath(new BezierLine(currentPose, scoringPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoringPose.getHeading())
                .build();
    }

    /**
     * Create an advanced curved approach for ball pickup
     * Generates a smooth S-curve for precise alignment
     */
    public PathChain createCurvedBallApproach(Pose currentPose, BallTarget target) {
        Pose ballPose = target.toPose();

        // Calculate approach geometry
        double angle = Math.atan2(
                ballPose.getY() - currentPose.getY(),
                ballPose.getX() - currentPose.getX()
        );

        // Create control point for smooth curve
        double midX = (currentPose.getX() + ballPose.getX()) / 2;
        double midY = (currentPose.getY() + ballPose.getY()) / 2;
        Pose controlPoint = new Pose(midX, midY, angle);

        Pose approachPose = new Pose(
                ballPose.getX() - approachDistance * Math.cos(angle),
                ballPose.getY() - approachDistance * Math.sin(angle),
                angle
        );

        // Create bezier curve path
        return follower.pathBuilder()
                .addPath(new BezierCurve(currentPose, controlPoint, approachPose))
                .setConstantHeadingInterpolation(angle)
                .build();
    }

    /**
     * Sort balls using nearest neighbor algorithm
     * Minimizes total travel distance
     */
    private List<BallTarget> nearestNeighborSort(Pose startPose, List<BallTarget> balls) {
        List<BallTarget> sorted = new ArrayList<>();
        List<BallTarget> remaining = new ArrayList<>(balls);

        Pose current = startPose;

        while (!remaining.isEmpty()) {
            BallTarget nearest = null;
            double minDistance = Double.MAX_VALUE;
            int nearestIndex = 0;

            // Find nearest ball
            for (int i = 0; i < remaining.size(); i++) {
                BallTarget ball = remaining.get(i);
                double distance = ball.distanceTo(current);

                if (distance < minDistance) {
                    minDistance = distance;
                    nearest = ball;
                    nearestIndex = i;
                }
            }

            if (nearest != null) {
                sorted.add(nearest);
                remaining.remove(nearestIndex);
                current = nearest.toPose();
            }
        }

        return sorted;
    }

    /**
     * Calculate total distance for a ball pickup sequence
     */
    public double calculateTotalDistance(Pose startPose, List<BallTarget> balls) {
        if (balls.isEmpty()) {
            return 0;
        }

        List<BallTarget> sortedBalls = nearestNeighborSort(startPose, balls);
        double totalDistance = 0;

        Pose current = startPose;
        for (BallTarget ball : sortedBalls) {
            totalDistance += distanceBetweenPoses(current, ball.toPose());
            current = ball.toPose();
        }

        // Add return distance
        if (scoringPose != null) {
            totalDistance += distanceBetweenPoses(current, scoringPose);
        }

        return totalDistance;
    }

    /**
     * Get estimated time to complete ball pickup sequence
     */
    public double estimatePickupTime(Pose startPose, List<BallTarget> balls, double robotSpeed) {
        double distance = calculateTotalDistance(startPose, balls);
        double timePerBall = 2.0; // seconds
        return (distance / robotSpeed) + (balls.size() * timePerBall);
    }

    /**
     * Helper: Distance between two poses
     */
    private double distanceBetweenPoses(Pose p1, Pose p2) {
        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
}
