package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Camera;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Intake;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.BallDetectionVision;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.AprilTagVision;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.BallPickupAndScoringSystem;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.AutonomousController;

public class LiveRobot extends Robot{
    public Follower follower;
    public Intake intake;
    public Turret turret;
    public Camera camera;
    public DualCamera dualCamera;
    
    // Autonomous ball pickup system components
    public BallDetectionVision ballDetectionVision;
    public AprilTagVision aprilTagVision;
    public BallPickupAndScoringSystem ballPickupSystem;
    public AutonomousController autonomousController;
    
    public static Pose lastPose = new Pose(0,0,Math.toRadians(0));
    {
        name = "CYPHER";
    }

    public LiveRobot(LinearOpMode opMode) {
        super(opMode);
        follower    = Constants.createFollower(hwmap);
        intake      = new Intake(this);
        turret      = new Turret(this);
      //  camera      = new Camera(this);
    }
    
    /**
     * Initialize autonomous ball pickup system components
     * Call this during OpMode init if autonomous ball pickup is needed
     */
    public void initializeAutonomousSystem() {
        try {
            // Initialize ball detection vision
            ballDetectionVision = new BallDetectionVision(this);
            ballDetectionVision.registerHardware(hwmap);
            
            // Initialize AprilTag vision for pose fusion
            aprilTagVision = new AprilTagVision(this);
            aprilTagVision.registerHardware(hwmap);
            
            // Create ball pickup and scoring system
            ballPickupSystem = new BallPickupAndScoringSystem(this, ballDetectionVision, aprilTagVision);
            
        } catch (Exception e) {
            addData("ERROR", "Failed to initialize autonomous system: " + e.getMessage());
        }
    }
    
    /**
     * Initialize driver-controlled autonomous mode
     * Call after initializeAutonomousSystem()
     */
    public void initializeAutonomousController(com.qualcomm.robotcore.hardware.Gamepad gamepad1, 
                                                com.qualcomm.robotcore.hardware.Gamepad gamepad2) {
        if (ballPickupSystem == null) {
            addData("ERROR", "Cannot initialize controller - system not initialized");
            return;
        }
        
        try {
            autonomousController = new AutonomousController(this, ballPickupSystem, gamepad1, gamepad2);
        } catch (Exception e) {
            addData("ERROR", "Failed to initialize autonomous controller: " + e.getMessage());
        }
    }

    @Override
    public void update(){
        super.update();
        follower.update();
       lastPose = follower.getPose();
    }
    public void startup(){
        isRed = true;
    }
    @Override
    public  void updateTelemetry(){
        super.updateTelemetry();
    }
    public void setLastPose(Pose p){
        lastPose = p;
    }
    public Pose getLastPose(){
        return lastPose;
    }

    /**
     * Get current robot pose from vision or follower
     */
    public Pose getRobotPose() {
        // Try to get pose from DualCamera first (AprilTag-based)
        if (dualCamera != null) {
            Pose cameraPose = dualCamera.getPose();
            if (cameraPose != null) {
                return cameraPose;
            }
        }

        // Fall back to follower pose
        return follower.getPose();
    }
}
