package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.BallDetectionVision;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.AprilTagVision;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.BallPickupAndScoringSystem;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.AutonomousController;

/**
 * Autonomous Ball Pickup and Scoring - Main OpMode
 * 
 * Features:
 * - Real-time ball detection via Husky Lens
 * - Field localization via AprilTag vision
 * - Autonomous ball pickup and scoring
 * - Driver-controlled toggle (Y button)
 * - Emergency stop (Back button)
 * - Haptic feedback for mode changes
 * - Full telemetry and debug output
 * 
 * Control Scheme:
 * - gamepad1.y: Toggle autonomous mode ON/OFF
 * - gamepad1.x: Override/cancel autonomous (emergency)
 * - gamepad1.back: Emergency stop (all systems halt)
 * 
 * @author 5143 Xcentrics
 */
@Autonomous(name = "Auto Ball Pickup & Scoring (5143)", group = "Autonomous")
public class AutoBallPickupAndScore extends LinearOpMode {
    
    private LiveRobot robot;
    private BallDetectionVision ballVision;
    private AprilTagVision aprilTagVision;
    private BallPickupAndScoringSystem autoSystem;
    private AutonomousController autoController;
    
    private boolean systemsInitialized = false;
    private long initStartTime = 0;
    private static final long INIT_TIMEOUT_MS = 5000; // 5 second init timeout
    
    @Override
    public void runOpMode() {
        try {
            // Initialize systems
            initializeSystems();
            
            // Wait for start
            waitForStart();
            
            if (isStopRequested()) {
                shutdown();
                return;
            }
            
            // Main autonomous loop
            runAutonomousLoop();
            
        } catch (Exception e) {
            telemetry.addData("FATAL ERROR", e.getMessage());
            telemetry.update();
            shutdown();
        }
    }
    
    /**
     * Initialize all robot systems and vision components
     */
    private void initializeSystems() {
        try {
            telemetry.addLine("Initializing systems...");
            telemetry.update();
            
            // Create robot instance
            robot = new LiveRobot(this);
            LiveRobot.isAuto = true;
            
            // Initialize ball detection vision
            telemetry.addLine("→ Initializing ball detection...");
            telemetry.update();
            ballVision = new BallDetectionVision(robot);
            ballVision.registerHardware(hardwareMap);
            
            // Initialize AprilTag vision
            telemetry.addLine("→ Initializing AprilTag vision...");
            telemetry.update();
            aprilTagVision = new AprilTagVision(robot);
            aprilTagVision.registerHardware(hardwareMap);
            
            // Create autonomous system
            telemetry.addLine("→ Creating autonomous system...");
            telemetry.update();
            autoSystem = new BallPickupAndScoringSystem(robot, ballVision, aprilTagVision);
            
            // Create driver controller
            telemetry.addLine("→ Creating driver controller...");
            telemetry.update();
            autoController = new AutonomousController(robot, autoSystem, gamepad1, gamepad2);
            robot.autonomousController = autoController;
            
            // Startup all systems
            telemetry.addLine("→ Starting up components...");
            telemetry.update();
            robot.startup();
            robot.intake.startup();
            robot.turret.startup();
            ballVision.startup();
            aprilTagVision.startup();
            autoSystem.startup();
            autoController.startup();
            
            systemsInitialized = true;
            
            telemetry.addLine("✓ All systems initialized successfully");
            telemetry.addLine("");
            telemetry.addLine("READY TO START");
            telemetry.addLine("- Press Y to enable autonomous mode");
            telemetry.addLine("- Press X to override (cancel auto)");
            telemetry.addLine("- Press Back for emergency stop");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("INIT ERROR", e.getMessage());
            telemetry.addLine("Stack: " + e.toString());
            telemetry.update();
            throw e;
        }
    }
    
    /**
     * Main autonomous execution loop
     */
    private void runAutonomousLoop() {
        if (!systemsInitialized) {
            telemetry.addLine("ERROR: Systems not initialized");
            telemetry.update();
            return;
        }
        
        telemetry.addLine("Autonomous mode started");
        telemetry.update();
        
        while (opModeIsActive() && !isStopRequested()) {
            try {
                // Update all robot systems
                robot.update();
                ballVision.update(this);
                aprilTagVision.update(this);
                autoSystem.update(this);
                autoController.update(this);
                
                // Update telemetry
                updateTelemetry();
                
            } catch (Exception e) {
                telemetry.addData("LOOP ERROR", e.getMessage());
                telemetry.addLine("Attempting recovery...");
                
                // Try to safely stop autonomous if error occurs
                try {
                    autoController.forceDisableAuto();
                } catch (Exception e2) {
                    // Silent recovery attempt failure
                }
                
                telemetry.update();
            }
        }
        
        shutdown();
    }
    
    /**
     * Update telemetry with current status
     */
    private void updateTelemetry() {
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("     AUTONOMOUS BALL PICKUP & SCORE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("");
        
        // Display status banner
        autoController.displayAutoStatusBanner();
        telemetry.addLine("");
        
        // Vision status
        telemetry.addLine("VISION STATUS:");
        telemetry.addData("  Ball Detection", ballVision.isConnected() ? "✓" : "✗");
        telemetry.addData("  Balls Detected", ballVision.getBallCount());
        telemetry.addData("  AprilTag Vision", aprilTagVision.isPoseValid() ? "✓" : "✗");
        telemetry.addData("  Tags Visible", 
            (aprilTagVision.isRedTagVisible() ? "R" : "-") + 
            (aprilTagVision.isBlueTagVisible() ? "B" : "-"));
        telemetry.addLine("");
        
        // Autonomous system status
        telemetry.addLine("SYSTEM STATUS:");
        telemetry.addData("  Current State", autoSystem.getCurrentState().toString());
        telemetry.addData("  Auto Enabled", autoSystem.isAutoEnabled());
        
        if (autoSystem.isAutoEnabled()) {
            telemetry.addLine("");
            telemetry.addLine("CONTROL BUTTONS:");
            telemetry.addLine("  Y = Toggle Auto (" + (gamepad1.y ? "PRESSED" : "ready") + ")");
            telemetry.addLine("  X = Override (" + (gamepad1.x ? "PRESSED" : "ready") + ")");
            telemetry.addLine("  Back = E-Stop (" + (gamepad1.back ? "PRESSED" : "ready") + ")");
        }
        
        // Detailed component status
        if (autoSystem.isAutoEnabled()) {
            telemetry.addLine("");
            telemetry.addLine("COMPONENT DETAILS:");
            robot.updateTelemetry(telemetry);
            ballVision.updateTelemetry(telemetry);
            aprilTagVision.updateTelemetry(telemetry);
            autoSystem.updateTelemetry(telemetry);
            autoController.updateTelemetry(telemetry);
        }
        
        telemetry.addLine("");
        telemetry.addLine("Loop Time: " + Math.round(getRuntime() * 1000) + "ms");
        telemetry.update();
    }
    
    /**
     * Safely shutdown all systems
     */
    private void shutdown() {
        try {
            if (robot != null) {
                // Disable autonomous
                if (autoController != null) {
                    autoController.forceDisableAuto();
                }
                
                // Stop mechanisms
                if (robot.intake != null) {
                    robot.intake.stopIntake();
                }
                if (robot.turret != null) {
                    robot.turret.stopAim();
                }
                if (robot.follower != null) {
                    robot.follower.breakFollowing();
                }
                
                // Shutdown components
                if (ballVision != null) {
                    ballVision.shutdown();
                }
                if (aprilTagVision != null) {
                    aprilTagVision.shutdown();
                }
                if (robot != null) {
                    robot.shutdown();
                }
            }
            
            telemetry.addLine("System shutdown complete");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("Shutdown Error", e.getMessage());
            telemetry.update();
        }
    }
}
