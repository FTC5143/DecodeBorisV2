package org.firstinspires.ftc.teamcode.xcentrics.components;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;

@SuppressWarnings("ALL")
public abstract class Component {

    protected final static int STATUS_OFFLINE = 0;
    protected final static int STATUS_ONLINE = 1;

    // The name of the Component, used in telemetry
    protected String name = "Component";

    // The string that appears in telemetry
    protected int status = STATUS_OFFLINE;

    // The robot that we are a part of
    @SuppressWarnings("CanBeFinal")
    protected Robot robot;
    protected Telemetry telemetry;

    @SuppressWarnings("unused")
    protected final DecimalFormat TELEMETRY_DECIMAL = new DecimalFormat("##.00");

    @SuppressWarnings("unused")
    public Component(Robot robot) {
        this.robot = robot;
        robot.registerComponent(this);
    }

    public void registerHardware(HardwareMap hwmap) {
        /**
         * Where all hardware used in the component is registered to its respective variable
         */
    }

    public void update(OpMode opmode) {
        /**
         * Called every time the robot update method is called
         */
    }

    public void startup() {
        /**
         * Called when robot.startup() is called, which should be called when an opmode is started
         */
        status = STATUS_ONLINE;
    }

    public void shutdown() {
        /**
         * Called when robot.shutdown() is called, which should be called when an opmode is stopped
         */
        status = STATUS_OFFLINE;
    }

    public void updateTelemetry(Telemetry telemetry) {
        /**
         * Called on every update. Used for outputting information about the component to the phone for debug purposes
         */
        this.telemetry = telemetry;
        addData("[CMP "+name+"]", status == STATUS_ONLINE ? "ONLINE": "OFFLINE");
    }
    public void addData(String caption,Object vaule){
        telemetry.addData(caption,vaule);
        robot.panelsTelemetry.addData(caption,vaule);
    }
    public void addLine(String caption){
        telemetry.addLine(caption);
        robot.panelsTelemetry.addLine(caption);
    }
    // internal time tracking
    private volatile long startTime = 0; // in nanoseconds
    public void halt(double seconds) {
        resetRuntime();
        while (getRuntime() < seconds) {
            robot.update();
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
