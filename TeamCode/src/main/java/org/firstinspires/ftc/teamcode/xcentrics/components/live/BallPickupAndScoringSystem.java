package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Autonomous Ball Pickup and Scoring System
 * State machine controlling ball detection, pickup, and scoring with turret aim
 * @author 5143 Xcentrics
 */
@Configurable
class AutoSystemConfig {
    // Ball detection parameters
    public static double ballDetectionTimeout = 3.0; // seconds to search for ball
    public static double ballPickupDistance = 8.0; // inches - when to trigger intake
    public static double ballStaleTimeout = 0.7; // seconds before a detection is considered stale
    public static double ballLostTimeout = 1.0; // seconds to wait for a missing ball before skipping
    public static double pickupDuration = 0.6; // seconds

    // Grouping parameters
    public static double groupingDistance = 15.0; // inches
    public static double groupBallWeight = 6.0;
    public static double groupDistanceWeight = 1.0;
    public static double groupCommitTimeout = 6.0; // seconds
    
    // Navigation parameters
    public static double pathFollowingTolerance = 2.0; // inches
    public static double pathFollowingTimeout = 10.0; // seconds per path
    
    // Turret parameters
    public static double turretPositionTolerance = 50; // encoder ticks
    public static double turretVelocityTolerance = 100; // RPM
    public static double turretAlignTimeout = 3.0; // seconds
    public static double flywheelSpinupTimeout = 2.0; // seconds
    public static double turretReadyNotifyCooldown = 1.0; // seconds

    // Flywheel velocity targets
    public static double closeRangeVelocity = 1300;
    public static double farRangeVelocity = 1600;
    public static double closeRangeDistance = 40.0; // inches
    
    // Goal positions (inches)
    public static double redGoalX = 135.0;
    public static double redGoalY = 141.0;
    public static double blueGoalX = 8.0;
    public static double blueGoalY = 142.0;
}

public class BallPickupAndScoringSystem extends Component {
    
    /**
     * Autonomous state machine states
     */
    public enum AutoState {
        MANUAL,              // Driver has full control, auto idle
        SCAN_AND_GROUP,      // Scan + cluster balls and select group
        COMMIT_TO_GROUP,     // Lock onto selected group and build collection order
        NAVIGATE_TO_BALL,    // Following Pedro path to ball position
        PICKUP,              // Executing intake sequence
        CHECK_GROUP_STATUS,  // Determine if more balls remain in group
        NAVIGATE_TO_GOAL,    // Moving to shooting position via AprilTag
        ALIGN_TURRET,        // Aiming turret and spinning up flywheel
        WAIT_FOR_READY,      // Polling turret status (±50 ticks, ±100 RPM)
        SHOOT,               // Executing shot sequence
        VERIFY,              // Confirming successful score
        ERROR                // Error state
    }

    /**
     * Field-positioned ball target
     */
    private static class BallTarget {
        long id;
        Pose fieldPose;
        double forwardInches;
        double rightInches;
        long lastSeenTime;

        BallTarget(long id, Pose fieldPose, double forwardInches, double rightInches, long lastSeenTime) {
            this.id = id;
            this.fieldPose = fieldPose;
            this.forwardInches = forwardInches;
            this.rightInches = rightInches;
            this.lastSeenTime = lastSeenTime;
        }
    }

    /**
     * Clustered ball group
     */
    private static class BallGroup {
        int groupId;
        List<BallTarget> balls = new ArrayList<>();
        Pose centroid;
        double score;

        int size() {
            return balls.size();
        }
    }
    
    private final LiveRobot robot;
    private AutoState currentState = AutoState.MANUAL;
    private AutoState previousState = AutoState.MANUAL;
    
    // Vision components
    private BallDetectionVision ballVision;
    private AprilTagVision aprilTagVision;
    
    // Timing
    private long stateStartTime = 0;
    private double stateTimeoutSeconds = 0;
    
    // Ball tracking
    private BallTarget targetBall = null;
    private PathChain ballPath = null;
    private PathChain goalPath = null;
    private final Map<Long, BallTarget> latestBallTargets = new HashMap<>();
    private final Map<Long, Long> ballLastSeen = new HashMap<>();
    private final Set<Long> collectedBallIds = new HashSet<>();
    private final Set<Long> skippedBallIds = new HashSet<>();
    private final List<BallDetectionVision.BallObservation> reusableObservations = new ArrayList<>();

    private List<BallGroup> currentGroups = new ArrayList<>();
    private BallGroup committedGroup = null;
    private List<BallTarget> committedBallOrder = new ArrayList<>();
    private int currentBallIndex = 0;
    private long groupCommitStartTime = 0;
    private long missingBallStartTime = 0;

    // Manual group selection index (optional)
    private int preferredGroupIndex = -1;
    
    // Shooting
    private Pose shootingPosition = null;
    private int shotsFired = 0;
    private static final int SHOTS_TO_FIRE = 3;
    private long lastReadyNotifyTime = 0;
    
    // Driver control state
    private boolean autoEnabled = false;
    private boolean previousAutoButtonState = false;
    
    {
        name = "Ball Pickup System";
    }
    
    public BallPickupAndScoringSystem(LiveRobot robot, BallDetectionVision ballVision, AprilTagVision aprilTagVision) {
        super(robot);
        this.robot = robot;
        this.ballVision = ballVision;
        this.aprilTagVision = aprilTagVision;
    }
    
    @Override
    public void startup() {
        super.startup();
        currentState = AutoState.MANUAL;
        autoEnabled = false;
        targetBall = null;
        ballPath = null;
        goalPath = null;
        latestBallTargets.clear();
        ballLastSeen.clear();
        collectedBallIds.clear();
        skippedBallIds.clear();
        currentGroups.clear();
        committedGroup = null;
        committedBallOrder.clear();
        currentBallIndex = 0;
        preferredGroupIndex = -1;
        reusableObservations.clear();
    }
    
    @Override
    public void update(LinearOpMode opMode) {
        super.update(opMode);
        
        if (ballVision == null || aprilTagVision == null) {
            currentState = AutoState.ERROR;
            return;
        }
        
        // Update AprilTag pose estimation
        Pose currentPose = aprilTagVision.getPose();
        if (currentPose != null && aprilTagVision.isPoseValid()) {
            robot.follower.setPose(currentPose);
        }
        
        // Handle state machine
        updateState(opMode);
    }
    
    /**
     * Main state machine update
     */
    private void updateState(LinearOpMode opMode) {
        switch (currentState) {
            case MANUAL:
                handleManualState(opMode);
                break;
            case SCAN_AND_GROUP:
                handleScanAndGroupState(opMode);
                break;
            case COMMIT_TO_GROUP:
                handleCommitToGroupState(opMode);
                break;
            case NAVIGATE_TO_BALL:
                handleNavigateToBallState(opMode);
                break;
            case PICKUP:
                handlePickupState(opMode);
                break;
            case CHECK_GROUP_STATUS:
                handleCheckGroupStatusState(opMode);
                break;
            case NAVIGATE_TO_GOAL:
                handleNavigateToGoalState(opMode);
                break;
            case ALIGN_TURRET:
                handleAlignTurretState(opMode);
                break;
            case WAIT_FOR_READY:
                handleWaitForReadyState(opMode);
                break;
            case SHOOT:
                handleShootState(opMode);
                break;
            case VERIFY:
                handleVerifyState(opMode);
                break;
            case ERROR:
                handleErrorState(opMode);
                break;
        }
        
        previousState = currentState;
    }
    
    // ============== STATE HANDLERS ==============
    
    private void handleManualState(LinearOpMode opMode) {
        // Driver has full control
        // Auto system is idle but still updating pose
        
        // Check for timeout - exit to SCAN_AND_GROUP if auto enabled
        if (autoEnabled && hasStateTimedOut()) {
            setState(AutoState.SCAN_AND_GROUP);
        }
    }
    
    private void handleScanAndGroupState(LinearOpMode opMode) {
        if (!autoEnabled) {
            setState(AutoState.MANUAL);
            return;
        }

        Pose currentPose = robot.follower.getPose();
        refreshBallObservations(currentPose);
        currentGroups = clusterBalls(getActiveBalls());

        if (currentGroups.isEmpty()) {
            if (hasStateTimedOut()) {
                addLine("AUTO: No ball groups found - rescanning");
                resetStateTimer();
            }
            return;
        }

        BallGroup selected = selectBestGroup(currentPose);
        if (selected == null) {
            return;
        }

        committedGroup = selected;
        committedBallOrder = buildCollectionOrder(currentPose, selected.balls);
        currentBallIndex = 0;
        groupCommitStartTime = System.currentTimeMillis();
        setState(AutoState.COMMIT_TO_GROUP);
    }

    private void handleCommitToGroupState(LinearOpMode opMode) {
        if (!autoEnabled) {
            stopAutonomous();
            setState(AutoState.MANUAL);
            return;
        }

        if (committedGroup == null || committedBallOrder.isEmpty()) {
            setState(AutoState.SCAN_AND_GROUP);
            return;
        }

        if (getGroupCommitElapsedTime() > AutoSystemConfig.groupCommitTimeout) {
            addLine("AUTO: Group commit timeout - re-evaluating");
            clearCommittedGroup();
            setState(AutoState.SCAN_AND_GROUP);
            return;
        }

        targetBall = getCurrentTargetBall();
        if (targetBall == null) {
            setState(AutoState.CHECK_GROUP_STATUS);
            return;
        }

        setState(AutoState.NAVIGATE_TO_BALL);
    }
    
    private void handleNavigateToBallState(LinearOpMode opMode) {
        if (!autoEnabled) {
            stopAutonomous();
            setState(AutoState.MANUAL);
            return;
        }

        if (previousState != AutoState.NAVIGATE_TO_BALL) {
            missingBallStartTime = 0;
        }
        
        Pose currentPose = robot.follower.getPose();
        refreshBallObservations(currentPose);

        if (targetBall == null) {
            targetBall = getCurrentTargetBall();
        }

        if (targetBall == null) {
            setState(AutoState.CHECK_GROUP_STATUS);
            return;
        }

        // Update target with latest detection if available
        BallTarget refreshed = latestBallTargets.get(targetBall.id);
        if (refreshed != null) {
            targetBall = refreshed;
            missingBallStartTime = 0;
        } else {
            if (missingBallStartTime == 0) {
                missingBallStartTime = System.currentTimeMillis();
            }
            if (getMissingBallElapsedTime() > AutoSystemConfig.ballLostTimeout) {
                addLine("AUTO: Target ball lost - skipping");
                skippedBallIds.add(targetBall.id);
                targetBall = null;
                ballPath = null;
                setState(AutoState.CHECK_GROUP_STATUS);
                return;
            }
        }

        Pose ballWorldPos = targetBall.fieldPose;

        // Create path to ball
        if (ballPath == null) {
            ballPath = buildPathToBall(currentPose, ballWorldPos);
            if (ballPath != null) {
                robot.follower.followPath(ballPath);
            }
        }

        // Check if reached ball
        if (robot.follower.getCurrentTValue() > 0.9 || currentPose.distanceFrom(ballWorldPos) < AutoSystemConfig.ballPickupDistance) {
            setState(AutoState.PICKUP);
        } else if (hasStateTimedOut()) {
            addLine("AUTO: Failed to reach ball - timeout");
            stopAutonomous();
            targetBall = null;
            ballPath = null;
            setState(AutoState.CHECK_GROUP_STATUS);
        }
    }
    
    private void handlePickupState(LinearOpMode opMode) {
        if (!autoEnabled) {
            robot.intake.stopIntake();
            setState(AutoState.MANUAL);
            return;
        }
        
        // Execute intake sequence
        if (previousState != AutoState.PICKUP) {
            // Just entered state
            robot.intake.intake();
            resetStateTimer();
        }
        
        // Wait for intake to complete (check for timeout or sensor feedback)
        if (getStateElapsedTime() > AutoSystemConfig.pickupDuration) {
            robot.intake.stopIntake();
            if (targetBall != null) {
                collectedBallIds.add(targetBall.id);
            }
            targetBall = null;
            ballPath = null;
            setState(AutoState.CHECK_GROUP_STATUS);
        }
    }

    private void handleCheckGroupStatusState(LinearOpMode opMode) {
        if (!autoEnabled) {
            stopAutonomous();
            setState(AutoState.MANUAL);
            return;
        }

        if (getGroupCommitElapsedTime() > AutoSystemConfig.groupCommitTimeout) {
            clearCommittedGroup();
            setState(AutoState.SCAN_AND_GROUP);
            return;
        }

        if (committedGroup == null || committedBallOrder.isEmpty()) {
            setState(AutoState.SCAN_AND_GROUP);
            return;
        }

        BallTarget next = getCurrentTargetBall();
        if (next != null) {
            targetBall = next;
            ballPath = null;
            setState(AutoState.NAVIGATE_TO_BALL);
            return;
        }

        // All balls in group collected or skipped
        setState(AutoState.NAVIGATE_TO_GOAL);
    }
    
    private void handleNavigateToGoalState(LinearOpMode opMode) {
        if (!autoEnabled) {
            stopAutonomous();
            setState(AutoState.MANUAL);
            return;
        }
        
        Pose currentPose = robot.follower.getPose();
        
        // Calculate optimal shooting position near goal using AprilTag
        if (shootingPosition == null) {
            shootingPosition = calculateOptimalShootingPosition(currentPose);
        }
        
        // Create and follow path to shooting position
        if (goalPath == null) {
            goalPath = buildPathToGoal(currentPose, shootingPosition);
            if (goalPath != null) {
                robot.follower.followPath(goalPath);
            }
        }
        
        // Check if reached shooting position
        if (currentPose.distanceFrom(shootingPosition) < AutoSystemConfig.pathFollowingTolerance) {
            setState(AutoState.ALIGN_TURRET);
        } else if (hasStateTimedOut()) {
            addLine("AUTO: Failed to reach goal - timeout");
            stopAutonomous();
            setState(AutoState.SCAN_AND_GROUP);
        }
    }
    
    private void handleAlignTurretState(LinearOpMode opMode) {
        if (!autoEnabled) {
            robot.turret.stopAim();
            setState(AutoState.MANUAL);
            return;
        }
        
        if (previousState != AutoState.ALIGN_TURRET) {
            // Just entered state - enable turret aiming
            Pose currentPose = robot.follower.getPose();
            Pose goalPose = getGoalPose();
            double goalAngle = Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());
            robot.turret.setTargetAngle(goalAngle);

            double distance = currentPose.distanceFrom(goalPose);
            double velocity = distance <= AutoSystemConfig.closeRangeDistance ?
                    AutoSystemConfig.closeRangeVelocity : AutoSystemConfig.farRangeVelocity;
            robot.turret.setFlywheelVelocity(velocity);
            resetStateTimer();
        }
        
        // Check if turret is ready
        if (isTurretReady()) {
            setState(AutoState.WAIT_FOR_READY);
        } else if (hasStateTimedOut()) {
            addLine("AUTO: Turret align timeout");
            stopAutonomous();
            setState(AutoState.SCAN_AND_GROUP);
        }
    }
    
    private void handleWaitForReadyState(LinearOpMode opMode) {
        if (!autoEnabled) {
            robot.turret.stopAim();
            setState(AutoState.MANUAL);
            return;
        }
        
        // Poll turret status
        if (isTurretReady()) {
            maybeNotifyReadyToFire();
            setState(AutoState.SHOOT);
            shotsFired = 0;
        } else if (hasStateTimedOut()) {
            addLine("AUTO: Turret ready timeout");
            stopAutonomous();
            setState(AutoState.SCAN_AND_GROUP);
        }
    }
    
    private void handleShootState(LinearOpMode opMode) {
        if (!autoEnabled) {
            robot.turret.stopAim();
            setState(AutoState.MANUAL);
            return;
        }
        
        if (previousState != AutoState.SHOOT) {
            // Just entered - fire shot
            robot.turret.shoot();
            shotsFired++;
            resetStateTimer();
        }
        
        // Fire multiple shots
        if (shotsFired < SHOTS_TO_FIRE && getStateElapsedTime() > 0.5) {
            if (isTurretReady()) {
                robot.turret.shoot();
                shotsFired++;
                resetStateTimer();
            }
        } else if (shotsFired >= SHOTS_TO_FIRE) {
            setState(AutoState.VERIFY);
        } else if (hasStateTimedOut()) {
            setState(AutoState.VERIFY);
        }
    }
    
    private void handleVerifyState(LinearOpMode opMode) {
        // Verify ball was scored
        robot.turret.stopAim();
        
        // Brief delay for feedback
        if (getStateElapsedTime() > 1.0) {
            // Return to scanning for next ball
            targetBall = null;
            shootingPosition = null;
            ballPath = null;
            goalPath = null;
            shotsFired = 0;
            clearCommittedGroup();
            setState(AutoState.SCAN_AND_GROUP);
        }
    }
    
    private void handleErrorState(LinearOpMode opMode) {
        // Error recovery
        stopAutonomous();
        addLine("AUTO: ERROR STATE - stopping all systems");
        
        if (hasStateTimedOut()) {
            setState(AutoState.MANUAL);
        }
    }
    
    // ============== HELPER METHODS ==============
    
    /**
     * Check if turret is ready to fire (position ±50 ticks, velocity ±100 RPM)
     */
    private boolean isTurretReady() {
        if (robot.turret == null) {
            return false;
        }
        return robot.turret.isReadyToFire(AutoSystemConfig.turretPositionTolerance,
                AutoSystemConfig.turretVelocityTolerance);
    }
    
    /**
     * Calculate ball position angle relative to robot
     */
    private double calculateBallAngleFromRobot() {
        if (targetBall == null) {
            return 0;
        }
        
        // Frame center is 160 pixels (half of 320 width)
        // Positive angle = right, negative = left
        double pixelOffset = targetBall.x - 160.0;
        
        // Approximate: ~0.5 degrees per pixel (tune for your camera FOV)
        return Math.toRadians(pixelOffset * 0.5);
    }
    
    /**
     * Calculate optimal shooting position based on AprilTag
     */
    private Pose calculateOptimalShootingPosition(Pose currentPose) {
        Pose goal = robot.isRed() ?
            new Pose(AutoSystemConfig.redGoalX, AutoSystemConfig.redGoalY, 0) :
            new Pose(AutoSystemConfig.blueGoalX, AutoSystemConfig.blueGoalY, 0);
        
        // Position ~24 inches away from goal
        double distance = 24.0;
        double angle = Math.atan2(goal.getY() - currentPose.getY(), goal.getX() - currentPose.getX());
        
        return new Pose(
            goal.getX() - distance * Math.cos(angle),
            goal.getY() - distance * Math.sin(angle),
            angle
        );
    }

    /**
     * Get goal pose based on alliance (fallback to static goal positions)
     */
    private Pose getGoalPose() {
        return robot.isRed() ?
                new Pose(AutoSystemConfig.redGoalX, AutoSystemConfig.redGoalY, 0) :
                new Pose(AutoSystemConfig.blueGoalX, AutoSystemConfig.blueGoalY, 0);
    }

    /**
     * Refresh latest ball observations and timestamps
     */
    private void refreshBallObservations(Pose currentPose) {
        latestBallTargets.clear();
        reusableObservations.clear();
        ballVision.getBallObservations(currentPose, reusableObservations);
        long now = System.currentTimeMillis();

        for (BallDetectionVision.BallObservation obs : reusableObservations) {
            BallTarget target = new BallTarget(
                    obs.id,
                    obs.fieldPose,
                    obs.forwardInches,
                    obs.rightInches,
                    now
            );
            latestBallTargets.put(obs.id, target);
            ballLastSeen.put(obs.id, now);
        }
    }

    /**
     * Get active balls (not collected, not stale)
     */
    private List<BallTarget> getActiveBalls() {
        List<BallTarget> active = new ArrayList<>();
        long now = System.currentTimeMillis();
        long staleMs = (long) (AutoSystemConfig.ballStaleTimeout * 1000);

        for (BallTarget target : latestBallTargets.values()) {
            if (collectedBallIds.contains(target.id) || skippedBallIds.contains(target.id)) {
                continue;
            }
            Long lastSeen = ballLastSeen.get(target.id);
            if (lastSeen != null && now - lastSeen <= staleMs) {
                active.add(target);
            }
        }

        return active;
    }

    /**
     * Cluster balls using simple distance-based grouping
     */
    private List<BallGroup> clusterBalls(List<BallTarget> balls) {
        List<BallGroup> groups = new ArrayList<>();
        Set<Long> visited = new HashSet<>();

        for (BallTarget ball : balls) {
            if (visited.contains(ball.id)) {
                continue;
            }

            BallGroup group = new BallGroup();
            ArrayDeque<BallTarget> queue = new ArrayDeque<>();
            queue.addLast(ball);
            visited.add(ball.id);

            while (!queue.isEmpty()) {
                BallTarget current = queue.removeFirst();
                group.balls.add(current);

                for (BallTarget other : balls) {
                    if (visited.contains(other.id)) {
                        continue;
                    }
                    double dist = current.fieldPose.distanceFrom(other.fieldPose);
                    if (dist <= AutoSystemConfig.groupingDistance) {
                        visited.add(other.id);
                        queue.addLast(other);
                    }
                }
            }

            group.centroid = computeCentroid(group.balls);
            group.groupId = computeGroupId(group.balls);
            groups.add(group);
        }

        return groups;
    }

    private Pose computeCentroid(List<BallTarget> balls) {
        if (balls.isEmpty()) {
            return new Pose(0, 0, 0);
        }
        double sumX = 0;
        double sumY = 0;
        for (BallTarget ball : balls) {
            sumX += ball.fieldPose.getX();
            sumY += ball.fieldPose.getY();
        }
        return new Pose(sumX / balls.size(), sumY / balls.size(), 0);
    }

    private int computeGroupId(List<BallTarget> balls) {
        List<Long> ids = new ArrayList<>();
        for (BallTarget ball : balls) {
            ids.add(ball.id);
        }
        Collections.sort(ids);
        int hash = 1;
        for (Long id : ids) {
            hash = 31 * hash + (int) (id ^ (id >>> 32));
        }
        return hash;
    }

    /**
     * Select the highest scoring group (or preferred manual group)
     */
    private BallGroup selectBestGroup(Pose currentPose) {
        if (currentGroups.isEmpty()) {
            return null;
        }

        // Score all groups
        for (BallGroup group : currentGroups) {
            double distance = currentPose.distanceFrom(group.centroid);
            group.score = (group.size() * AutoSystemConfig.groupBallWeight)
                    - (distance * AutoSystemConfig.groupDistanceWeight);
        }

        // Manual selection (if set)
        if (preferredGroupIndex >= 0) {
            int clamped = Math.max(0, Math.min(preferredGroupIndex, currentGroups.size() - 1));
            preferredGroupIndex = clamped;
            return currentGroups.get(clamped);
        }

        // Auto selection
        currentGroups.sort(Comparator
                .comparingDouble((BallGroup g) -> g.score).reversed()
                .thenComparingInt(BallGroup::size).reversed());

        return currentGroups.get(0);
    }

    /**
     * Build nearest-neighbor collection order
     */
    private List<BallTarget> buildCollectionOrder(Pose startPose, List<BallTarget> balls) {
        List<BallTarget> remaining = new ArrayList<>(balls);
        List<BallTarget> ordered = new ArrayList<>();
        Pose current = startPose;

        while (!remaining.isEmpty()) {
            BallTarget nearest = null;
            double bestDist = Double.MAX_VALUE;
            for (BallTarget ball : remaining) {
                double dist = current.distanceFrom(ball.fieldPose);
                if (dist < bestDist) {
                    bestDist = dist;
                    nearest = ball;
                }
            }
            if (nearest == null) {
                break;
            }
            ordered.add(nearest);
            current = nearest.fieldPose;
            remaining.remove(nearest);
        }

        return ordered;
    }

    /**
     * Get next target ball in committed order
     */
    private BallTarget getCurrentTargetBall() {
        while (currentBallIndex < committedBallOrder.size()) {
            BallTarget next = committedBallOrder.get(currentBallIndex);
            if (collectedBallIds.contains(next.id) || skippedBallIds.contains(next.id)) {
                currentBallIndex++;
                continue;
            }
            return next;
        }
        return null;
    }

    private void clearCommittedGroup() {
        committedGroup = null;
        committedBallOrder.clear();
        currentBallIndex = 0;
        groupCommitStartTime = 0;
        preferredGroupIndex = -1;
    }

    private double getGroupCommitElapsedTime() {
        if (groupCommitStartTime == 0) {
            return 0;
        }
        return (System.currentTimeMillis() - groupCommitStartTime) / 1000.0;
    }

    private double getMissingBallElapsedTime() {
        if (missingBallStartTime == 0) {
            return 0;
        }
        return (System.currentTimeMillis() - missingBallStartTime) / 1000.0;
    }

    private void maybeNotifyReadyToFire() {
        if (robot.autonomousController == null) {
            return;
        }
        long now = System.currentTimeMillis();
        long cooldownMs = (long) (AutoSystemConfig.turretReadyNotifyCooldown * 1000);
        if (now - lastReadyNotifyTime > cooldownMs) {
            robot.autonomousController.sendReadyToFireNotification();
            lastReadyNotifyTime = now;
        }
    }
    
    /**
     * Build path to ball position (requires Pedro Pathing setup)
     */
    private PathChain buildPathToBall(Pose start, Pose ballPose) {
        try {
            return robot.follower.pathBuilder()
                .addPath(new BezierLine(
                    new Point(start.getX(), start.getY(), Point.CARTESIAN),
                    new Point(ballPose.getX(), ballPose.getY(), Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(start.getHeading(), ballPose.getHeading())
                .build();
        } catch (Exception e) {
            addLine("ERROR building ball path: " + e.getMessage());
            return null;
        }
    }
    
    /**
     * Build path to goal position
     */
    private PathChain buildPathToGoal(Pose start, Pose goalPose) {
        try {
            return robot.follower.pathBuilder()
                .addPath(new BezierCurve(
                    new Point(start.getX(), start.getY(), Point.CARTESIAN),
                    new Point((start.getX() + goalPose.getX()) / 2, (start.getY() + goalPose.getY()) / 2, Point.CARTESIAN),
                    new Point(goalPose.getX(), goalPose.getY(), Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(start.getHeading(), goalPose.getHeading())
                .build();
        } catch (Exception e) {
            addLine("ERROR building goal path: " + e.getMessage());
            return null;
        }
    }
    
    /**
     * Stop all autonomous mechanisms safely
     */
    private void stopAutonomous() {
        robot.intake.stopIntake();
        robot.turret.stopAim();
        robot.follower.breakFollowing();
    }
    
    /**
     * Set state with automatic timing
     */
    private void setState(AutoState newState) {
        if (newState != currentState) {
            currentState = newState;
            resetStateTimer();
            
            // Set appropriate timeout based on state
            switch (newState) {
                case SCAN_AND_GROUP:
                    stateTimeoutSeconds = AutoSystemConfig.ballDetectionTimeout;
                    break;
                case COMMIT_TO_GROUP:
                    stateTimeoutSeconds = AutoSystemConfig.groupCommitTimeout;
                    break;
                case NAVIGATE_TO_BALL:
                case NAVIGATE_TO_GOAL:
                    stateTimeoutSeconds = AutoSystemConfig.pathFollowingTimeout;
                    break;
                case ALIGN_TURRET:
                    stateTimeoutSeconds = AutoSystemConfig.turretAlignTimeout;
                    break;
                case WAIT_FOR_READY:
                    stateTimeoutSeconds = AutoSystemConfig.flywheelSpinupTimeout;
                    break;
                case PICKUP:
                    stateTimeoutSeconds = AutoSystemConfig.pickupDuration;
                    break;
                default:
                    stateTimeoutSeconds = 10.0;
            }
        }
    }
    
    private void resetStateTimer() {
        stateStartTime = System.currentTimeMillis();
    }
    
    private double getStateElapsedTime() {
        return (System.currentTimeMillis() - stateStartTime) / 1000.0;
    }
    
    private boolean hasStateTimedOut() {
        return getStateElapsedTime() > stateTimeoutSeconds;
    }
    
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        
        addData("Auto Enabled", autoEnabled);
        addData("Current State", currentState.toString());
        addData("State Time", String.format("%.2f/%.2f s", getStateElapsedTime(), stateTimeoutSeconds));

        addData("Balls Detected", ballVision.getBallCount());
        addData("Active Groups", currentGroups.size());
        if (committedGroup != null) {
            addData("Committed Group", committedGroup.groupId + " (" + committedGroup.size() + ")");
        }
        if (preferredGroupIndex >= 0) {
            addData("Manual Group Index", preferredGroupIndex);
        }

        if (targetBall != null) {
            addData("Target Ball ID", targetBall.id);
            addData("Target Ball Pose", String.format("(%.1f, %.1f)",
                    targetBall.fieldPose.getX(), targetBall.fieldPose.getY()));
        }

        addData("Collected", collectedBallIds.size());
        addData("Skipped", skippedBallIds.size());
        addData("Shots Fired", shotsFired);
        addData("Tags Visible", (aprilTagVision.isRedTagVisible() ? "R" : "-") + (aprilTagVision.isBlueTagVisible() ? "B" : "-"));
        if (robot.turret != null) {
            addData("Turret Error", String.format("%.1f", robot.turret.getTurretError()));
            addData("Flywheel Error", String.format("%.1f", robot.turret.getFlywheelError()));
            addData("Ready To Fire", isTurretReady());
        }
    }
    
    // ============== PUBLIC API ==============
    
    public void setAutoEnabled(boolean enabled) {
        this.autoEnabled = enabled;
        if (enabled && currentState == AutoState.MANUAL) {
            setState(AutoState.SCAN_AND_GROUP);
        } else if (!enabled && currentState != AutoState.MANUAL) {
            stopAutonomous();
            setState(AutoState.MANUAL);
        }
    }
    
    public boolean isAutoEnabled() {
        return autoEnabled;
    }
    
    public AutoState getCurrentState() {
        return currentState;
    }

    /**
     * Cycle preferred group index for manual selection
     */
    public void cyclePreferredGroup(int delta) {
        if (currentGroups.isEmpty()) {
            return;
        }
        if (preferredGroupIndex < 0) {
            preferredGroupIndex = 0;
        }
        preferredGroupIndex += delta;
        if (preferredGroupIndex < 0) {
            preferredGroupIndex = currentGroups.size() - 1;
        }
        if (preferredGroupIndex >= currentGroups.size()) {
            preferredGroupIndex = 0;
        }
    }
}
