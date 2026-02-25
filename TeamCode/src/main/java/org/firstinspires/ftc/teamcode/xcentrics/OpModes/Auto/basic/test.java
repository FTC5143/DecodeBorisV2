package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.basic;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroDrawing;
import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
@Autonomous(name = "test")
public class test extends LiveAutoBase {
    private PedroDrawing pedroDrawing = new PedroDrawing();
    private int p = 0;
    PathChain Path1;

    @Override
    public void on_init() {
        pedroDrawing.init();
    }

    @Override
    public void on_start() {
        Path1 = robot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(72.000, 9.000),
                                new Pose(122.905, 51.482),
                                new Pose(20.903, 34.339),
                                new Pose(72.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))

                .build();
        robot.follower.setStartingPose(new Pose(72.000, 9,Math.toRadians(0)));
        followPath(Path1,1);
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        pedroDrawing.drawDebug(robot.follower);
    }
}
