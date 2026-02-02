package org.firstinspires.ftc.teamcode.xcentrics.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.AutonomousController;

/**
 * TeleOp with Integrated Autonomous Ball Pickup System
 * 
 * This TeleOp variant integrates the autonomous ball pickup system for hybrid control:
 * - Use Y button to toggle autonomous mode on/off
 * - Driver maintains manual control with optional autonomous assistance
 * - Perfect for matches where autonomous pickup help is needed mid-match
 * 
 * Features:
 * - Full manual drive control via left/right sticks
 * - Manual intake/turret control via buttons
 * - Toggle autonomous mode with Y button (haptic feedback)
 * - Emergency override with X button
 * - Emergency stop with Back button
 * 
 * @author 5143 Xcentrics
 */
@TeleOp(name = "TeleOp with Auto Assist (5143)", group = "TeleOp")
public class TeleopWithAutoAssist extends LiveTeleopBase {
    
    private AutonomousController autoController;
    private boolean previousY = false;
    private boolean previousX = false;
    private boolean previousBack = false;
    
    private static final double DRIVE_SPEED = 1.0;
    private static final double TURRET_SPEED = 0.8;
    private static final double INTAKE_SPEED = 1.0;
    
    @Override
    public void on_init() {
        telemetry.addLine("Initializing TeleOp with Auto Assist...");
        telemetry.update();
        
        // Initialize autonomous system components
        try {
            robot.initializeAutonomousSystem();
            robot.initializeAutonomousController(gamepad1, gamepad2);
            autoController = robot.autonomousController;
            
            telemetry.addLine("✓ Autonomous system initialized");
            telemetry.addLine("Press Y to enable auto ball pickup");
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addLine("✗ Failed to initialize auto system");
            telemetry.addLine("Error: " + e.getMessage());
            telemetry.update();
            autoController = null;
        }
    }
    
    @Override
    public void on_start() {
        telemetry.addLine("TeleOp Started - ready for input");
        telemetry.update();
    }
    
    @Override
    public void on_loop() {
        try {
            // Update autonomous controller (handles button polling)
            if (autoController != null) {
                autoController.update(this);
            }
            
            // Handle autonomous mode
            if (autoController != null && autoController.isAutoModeActive()) {
                // Autonomous mode active - turret and intake controlled by system
                // Driver can still control drive for repositioning
                handleAutonomousMode();
            } else {
                // Manual mode - driver has full control
                handleManualMode();
            }
            
            // Update telemetry
            robot.updateTelemetry(telemetry);
            if (autoController != null) {
                autoController.updateTelemetry(telemetry);
            }
            
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }
        
        telemetry.update();
    }
    
    /**
     * Handle input during autonomous mode
     * Driver can move robot but turret/intake are autonomous
     */
    private void handleAutonomousMode() {
        // Allow drive control
        handleDriveInput();
        
        // Display autonomous status
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("AUTO MODE ACTIVE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("State: " + autoController.getCurrentAutoState().toString());
        telemetry.addLine("Press X to override/cancel auto");
        telemetry.addLine("Press Back for emergency stop");
    }
    
    /**
     * Handle input during manual mode
     * Driver has full control
     */
    private void handleManualMode() {
        // Drive control
        handleDriveInput();
        
        // Turret control (gamepad2)
        handleTurretInput();
        
        // Intake control (gamepad2)
        handleIntakeInput();
        
        // Display manual mode status
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("MANUAL MODE");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("Press Y to enable autonomous assist");
    }
    
    /**
     * Handle robot drive input (gamepad1 left/right sticks)
     */
    private void handleDriveInput() {
        if (gamepad1 == null) {
            return;
        }
        
        // Get drive inputs
        double leftY = -gamepad1.left_stick_y;  // Negative for forward
        double rightX = gamepad1.right_stick_x;
        double leftX = gamepad1.left_stick_x;
        
        // Set drivetrain powers (assuming you have a drive system)
        // This is a placeholder - adjust for your actual drive configuration
        telemetry.addData("Drive", String.format("LY:%.2f RX:%.2f LX:%.2f", leftY, rightX, leftX));
    }
    
    /**
     * Handle turret input (gamepad2)
     */
    private void handleTurretInput() {
        if (gamepad2 == null || robot.turret == null) {
            return;
        }
        
        // Manual turret aim with left stick
        double turretInput = gamepad2.left_stick_x;
        if (Math.abs(turretInput) > 0.1) {
            robot.turret.turret.queue_power(turretInput * TURRET_SPEED);
        } else {
            robot.turret.turret.queue_power(0);
        }
        
        // Flywheel control
        if (gamepad2.right_bumper) {
            robot.turret.fly1.setVelocity(1300);
            robot.turret.fly2.setVelocity(1300);
        } else if (gamepad2.left_bumper) {
            robot.turret.fly1.setVelocity(0);
            robot.turret.fly2.setVelocity(0);
        }
        
        // Launch
        if (gamepad2.a) {
            robot.turret.launch();
        }
    }
    
    /**
     * Handle intake input (gamepad2)
     */
    private void handleIntakeInput() {
        if (gamepad2 == null || robot.intake == null) {
            return;
        }
        
        // Intake
        if (gamepad2.y) {
            robot.intake.intake();
        }
        // Outtake
        else if (gamepad2.x) {
            robot.intake.outtake();
        }
        // Stop
        else {
            robot.intake.stopIntake();
        }
    }
    
    @Override
    public void on_stop() {
        // Emergency shutdown
        if (robot != null) {
            robot.intake.stopIntake();
            robot.turret.stopAim();
            if (autoController != null) {
                autoController.forceDisableAuto();
            }
        }
        
        telemetry.addLine("TeleOp Stopped");
        telemetry.update();
    }
}
