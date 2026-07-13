package org.firstinspires.ftc.teamcode.xcentrics.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import java.util.concurrent.TimeUnit;


public class LiveRobot extends Robot{

    {
        name = "CYPHER";
    }

    public LiveRobot(LinearOpMode opMode) {
        super(opMode);
    }


    @Override
    public void update(){
        super.update();
    }
    public void startup(){
        isRed = true;
    }
    @Override
    public  void updateTelemetry(){
        super.updateTelemetry();
    }


    private volatile long startTime = 0; // in nanoseconds
    public void halt(double seconds) {
        resetRuntime();
        while (getRuntime() < seconds) {
            update();
        }
    }
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }
    public void resetRuntime() {
        startTime = System.nanoTime();
    }
}
