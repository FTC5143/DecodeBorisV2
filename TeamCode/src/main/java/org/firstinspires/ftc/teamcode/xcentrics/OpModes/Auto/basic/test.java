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
    PathChain Path2,Path3,Path4;

    @Override
    public void on_init() {
        pedroDrawing.init();
        robot.follower.setStartingPose(new Pose(9.161, 60.258,Math.toRadians(270)));
    }

    @Override
    public void on_start() {
        Path1 = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.161, 60.258),

                                new Pose(11.903, 21.419)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-156))

                .build();

        Path2 = robot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.903, 21.419),
                                new Pose(13.532, 18.806),
                                new Pose(10.710, 11.097)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(260))

                .build();


        Path3 = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.871, 34.484),

                                new Pose(14.516, 34.806)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = robot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.516, 34.806),

                                new Pose(56.711, 8.514)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        pedroDrawing.drawDebug(robot.follower);
        update();
        robot.intake.intake();
    }
    private void update(){
        switch(p){
            case 0:
                robot.intake.intake();
                followPath(Path1);
                p++;
                break;

            case 1:
                if(!robot.follower.isBusy()){
                    followPath(Path2);
                    p++;
                    break;
                }

            case 2:
                if(!robot.follower.isBusy()){
                    followPath(Path3);
                    p++;
                    break;
                }

            case 3:
                if(!robot.follower.isBusy()){
                    followPath(Path4);
                    p++;
                    break;
                }

            default:
                break;
        }
    }
}
