package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;


public abstract class LiveAutoBase extends LinearOpMode {

    protected LiveRobot robot;

    @Override
    public void runOpMode() {
        robot = new LiveRobot(this);
        // Start up the robot as soon as the program is initialized
        robot.startup();
        robot.turret.startup();
        robot.intake.startup();
        on_init();
        waitForStart();
        on_start();
        while(opModeIsActive() && !isStopRequested() && isStarted()) {
            on_loop();
            robot.follower.update();
            //robot.update();
        }
        on_stop();
        // Shut the robot down as soon as the program is finished
        robot.shutdown();
    }

    // Called when init is pressed, runs once
    public abstract void on_init();
    // Called when start is pressed, runs once
    public abstract void on_start();

    // Called when stop is pressed, runs once
    public abstract void on_stop();
    //looped
    public abstract void on_loop();


    protected void halt(double seconds) {
        resetRuntime();
        while (getRuntime() < seconds && opModeIsActive() && opModeIsActive() && !isStopRequested() && isStarted()) {
            robot.update();
        }
    }
    protected void followPath(PathChain p){
        robot.follower.followPath(p);
    }
    protected void followPath(PathChain p, double m){
        robot.follower.setMaxPower(m);
        robot.follower.followPath(p);
    }
}
