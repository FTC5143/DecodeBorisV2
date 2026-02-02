package org.firstinspires.ftc.teamcode.xcentrics.components.live;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;

/**
 * Autonomous Controller - Driver Input Handler
 * Manages toggle button input, haptic feedback, and autonomous mode switching
 * @author 5143 Xcentrics
 */
@Configurable
class AutonomousControlConfig {
    public static boolean enableHapticFeedback = true;
    public static double hapticDuration = 0.3; // seconds
    public static double emergencyStopRumbleIntensity = 1.0;
    public static double toggleOnRumbleIntensity = 0.5;
    public static double toggleOffRumbleIntensity = 0.3;
    public static boolean enableManualGroupSelection = true;
}

public class AutonomousController extends Component {
    
    private final LiveRobot robot;
    private final BallPickupAndScoringSystem autoSystem;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    
    // Button state tracking
    private boolean previousToggleButtonState = false;
    private boolean previousOverrideButtonState = false;
    private boolean previousEmergencyStopState = false;
    private boolean previousGroupLeftState = false;
    private boolean previousGroupRightState = false;
    
    // Haptic feedback timing
    private long lastHapticTime = 0;
    private volatile boolean hapticPending = false;
    private HapticPattern pendingHapticPattern = null;
    
    /**
     * Haptic feedback patterns
     */
    public enum HapticPattern {
        TOGGLE_ON(100, 100, 0.5),      // 100ms on, 100ms off, 50% intensity
        TOGGLE_OFF(50, 50, 0.3),        // 50ms on, 50ms off, 30% intensity
        AUTO_ENGAGED(100, 100, 0.4),    // Double pulse - auto engaged
        MODE_CHANGED(50, 150, 0.6),     // Quick double tap - mode changed
        EMERGENCY_STOP(200, 100, 1.0),  // Long pulse - emergency stop
        READY_TO_FIRE(100, 50, 0.5);    // Rapid pulse - ready notification
        
        public final int onMs;
        public final int offMs;
        public final double intensity;
        
        HapticPattern(int onMs, int offMs, double intensity) {
            this.onMs = onMs;
            this.offMs = offMs;
            this.intensity = intensity;
        }
    }
    
    {
        name = "Autonomous Controller";
    }
    
    public AutonomousController(LiveRobot robot, BallPickupAndScoringSystem autoSystem, 
                                 Gamepad gamepad1, Gamepad gamepad2) {
        super(robot);
        this.robot = robot;
        this.autoSystem = autoSystem;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
    
    @Override
    public void startup() {
        super.startup();
        previousToggleButtonState = false;
        previousOverrideButtonState = false;
        previousEmergencyStopState = false;
    }
    
    @Override
    public void update(LinearOpMode opMode) {
        super.update(opMode);
        
        // Poll input buttons
        updateToggleButton();
        updateOverrideButton();
        updateEmergencyStop();
        updateManualGroupSelection();
        
        // Process haptic feedback requests
        processHapticFeedback();
    }
    
    // ============== INPUT HANDLERS ==============
    
    /**
     * Handle auto mode toggle button (gamepad1.y)
     * Rising edge triggers toggle
     */
    private void updateToggleButton() {
        if (gamepad1 == null) {
            return;
        }
        
        boolean currentButtonState = gamepad1.y;
        
        // Rising edge detection
        if (currentButtonState && !previousToggleButtonState) {
            handleToggleButtonPress();
        }
        
        previousToggleButtonState = currentButtonState;
    }
    
    /**
     * Execute toggle action
     */
    private void handleToggleButtonPress() {
        boolean newAutoState = !autoSystem.isAutoEnabled();
        autoSystem.setAutoEnabled(newAutoState);
        
        // Haptic feedback
        if (AutonomousControlConfig.enableHapticFeedback) {
            if (newAutoState) {
                triggerHapticFeedback(HapticPattern.TOGGLE_ON);
                addLine("AUTO MODE: ENABLED (gamepad rumble)");
            } else {
                triggerHapticFeedback(HapticPattern.TOGGLE_OFF);
                addLine("AUTO MODE: DISABLED (gamepad rumble)");
            }
        }
    }
    
    /**
     * Handle driver override button (gamepad1.x)
     * Cancels/interrupts autonomous mode at any time
     */
    private void updateOverrideButton() {
        if (gamepad1 == null) {
            return;
        }
        
        boolean currentButtonState = gamepad1.x;
        
        // Rising edge detection
        if (currentButtonState && !previousOverrideButtonState) {
            handleOverrideButtonPress();
        }
        
        previousOverrideButtonState = currentButtonState;
    }
    
    /**
     * Execute override action - safely interrupt autonomous mode
     */
    private void handleOverrideButtonPress() {
        if (autoSystem.isAutoEnabled()) {
            addLine("DRIVER OVERRIDE: Canceling autonomous mode");
            triggerHapticFeedback(HapticPattern.MODE_CHANGED);
            autoSystem.setAutoEnabled(false);
        }
    }
    
    /**
     * Handle emergency stop (gamepad1.back)
     * Immediately stops all mechanisms
     */
    private void updateEmergencyStop() {
        if (gamepad1 == null) {
            return;
        }
        
        boolean currentButtonState = gamepad1.back;
        
        // Rising edge detection
        if (currentButtonState && !previousEmergencyStopState) {
            handleEmergencyStop();
        }
        
        previousEmergencyStopState = currentButtonState;
    }

    /**
     * Optional manual group selection (gamepad1.dpad_left/right)
     */
    private void updateManualGroupSelection() {
        if (!AutonomousControlConfig.enableManualGroupSelection || gamepad1 == null) {
            return;
        }

        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (left && !previousGroupLeftState) {
            autoSystem.cyclePreferredGroup(-1);
            triggerHapticFeedback(HapticPattern.MODE_CHANGED);
        }

        if (right && !previousGroupRightState) {
            autoSystem.cyclePreferredGroup(1);
            triggerHapticFeedback(HapticPattern.MODE_CHANGED);
        }

        previousGroupLeftState = left;
        previousGroupRightState = right;
    }
    
    /**
     * Execute emergency stop - all systems halt
     */
    private void handleEmergencyStop() {
        addLine("EMERGENCY STOP ACTIVATED");
        
        // Disable autonomous system
        autoSystem.setAutoEnabled(false);
        
        // Stop all mechanisms
        robot.intake.stopIntake();
        robot.turret.stopAim();
        robot.follower.breakFollowing();
        
        // Haptic feedback - strong warning rumble
        if (AutonomousControlConfig.enableHapticFeedback) {
            triggerHapticFeedback(HapticPattern.EMERGENCY_STOP);
        }
    }
    
    // ============== HAPTIC FEEDBACK ==============
    
    /**
     * Trigger a haptic feedback pattern on gamepad1
     */
    public void triggerHapticFeedback(HapticPattern pattern) {
        if (!AutonomousControlConfig.enableHapticFeedback || gamepad1 == null) {
            return;
        }
        
        pendingHapticPattern = pattern;
        hapticPending = true;
        lastHapticTime = System.currentTimeMillis();
    }
    
    /**
     * Process pending haptic feedback
     * Non-blocking - distributes rumble commands over multiple frames
     */
    private void processHapticFeedback() {
        if (!hapticPending || pendingHapticPattern == null || gamepad1 == null) {
            return;
        }
        
        long elapsedMs = System.currentTimeMillis() - lastHapticTime;
        long totalCycleDuration = pendingHapticPattern.onMs + pendingHapticPattern.offMs;
        long cyclePosition = elapsedMs % totalCycleDuration;
        
        if (cyclePosition < pendingHapticPattern.onMs) {
            // Rumble ON
            gamepad1.rumble(pendingHapticPattern.intensity, pendingHapticPattern.intensity, 50);
        } else {
            // Rumble OFF
            gamepad1.rumble(0, 0, 50);
        }
        
        // Stop after 3 cycles
        if (elapsedMs > totalCycleDuration * 3) {
            hapticPending = false;
            gamepad1.rumble(0, 0, 1);
        }
    }
    
    /**
     * Send ready notification to driver
     * Called when turret is ready to fire
     */
    public void sendReadyToFireNotification() {
        triggerHapticFeedback(HapticPattern.READY_TO_FIRE);
        addLine("READY TO FIRE - Haptic feedback sent");
    }
    
    // ============== STATUS REPORTING ==============
    
    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        
        // Auto mode status
        String autoStatus = autoSystem.isAutoEnabled() ? 
            "ON (" + autoSystem.getCurrentState().toString() + ")" : 
            "OFF";
        addData("Auto Mode", autoStatus);
        
        // Button indicators
        addData("Toggle Button (Y)", gamepad1 != null && gamepad1.y ? "PRESSED" : "");
        addData("Override Button (X)", gamepad1 != null && gamepad1.x ? "PRESSED" : "");
        addData("Emergency Stop (Back)", gamepad1 != null && gamepad1.back ? "PRESSED" : "");
        if (AutonomousControlConfig.enableManualGroupSelection) {
            addData("Group Select (DPad)", "←/→");
        }
        
        // Haptic status
        if (AutonomousControlConfig.enableHapticFeedback) {
            String hapticStatus = hapticPending ? 
                "ACTIVE (" + pendingHapticPattern.toString() + ")" : 
                "IDLE";
            addData("Haptic Feedback", hapticStatus);
        }
    }
    
    // ============== TELEMETRY HELPERS ==============
    
    /**
     * Display auto system status banner
     */
    public void displayAutoStatusBanner() {
        if (autoSystem.isAutoEnabled()) {
            addLine("╔════════════════════════════╗");
            addLine("║   AUTO MODE ACTIVE         ║");
            addLine("║   State: " + padString(autoSystem.getCurrentState().toString(), 16) + "║");
            addLine("║   Press X to override      ║");
            addLine("║   Press Back for E-Stop    ║");
            addLine("╚════════════════════════════╝");
        } else {
            addLine("┌────────────────────────────┐");
            addLine("│   Manual Control Active    │");
            addLine("│   Press Y to enable Auto   │");
            addLine("└────────────────────────────┘");
        }
    }
    
    private String padString(String str, int length) {
        if (str.length() >= length) {
            return str.substring(0, length);
        }
        int padding = length - str.length();
        int padLeft = padding / 2;
        int padRight = padding - padLeft;
        return " ".repeat(padLeft) + str + " ".repeat(padRight);
    }
    
    // ============== PUBLIC API ==============
    
    /**
     * Get current autonomous state
     */
    public BallPickupAndScoringSystem.AutoState getCurrentAutoState() {
        return autoSystem.getCurrentState();
    }
    
    /**
     * Check if autonomous mode is currently active
     */
    public boolean isAutoModeActive() {
        return autoSystem.isAutoEnabled();
    }
    
    /**
     * Force disable autonomous mode (for safety)
     */
    public void forceDisableAuto() {
        if (autoSystem.isAutoEnabled()) {
            autoSystem.setAutoEnabled(false);
            triggerHapticFeedback(HapticPattern.EMERGENCY_STOP);
        }
    }
    
    /**
     * Get reference to underlying autonomous system
     */
    public BallPickupAndScoringSystem getAutoSystem() {
        return autoSystem;
    }
}
