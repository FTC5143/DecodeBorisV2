package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import static org.firstinspires.ftc.teamcode.xcentrics.robots.Robot.isRed;

import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroDrawing;
import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Turret;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.blue.FarBlueAutoPaths;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.red.SmallTriangle;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.smallTriangleBlue;

@Autonomous(name = "Small Blue Triangle 9 Artifact",group = "blue")
public class Small9Blue extends LiveAutoBase {

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    FarBlueAutoPaths paths;
    public PathChain p1;
    public PathChain p2;
    public PathChain p3;
    public PathChain p4;
    public PathChain p5;
    public PathChain p6;
    public PathChain p7;
    public PathChain p8;
    private int p = 0;
    @Override
    public void on_init() {
        paths = new FarBlueAutoPaths(robot.follower);
        robot.turret.far();
        robot.follower.setStartingPose(new Pose(56.71148134046984, 8.514007076140375,Math.toRadians(90)));
        PedroDrawing.init();
        isRed = false;
        Follower follower = robot.follower;
        p1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.711, 8.514),
                                new Pose(62.034, 32.385),
                                new Pose(49.163, 34.256)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        p2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.163, 34.256),

                                new Pose(42.871, 34.484)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        p3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.871, 34.484),

                                new Pose(14.516, 34.806)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        p4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.516, 34.806),

                                new Pose(56.711, 8.514)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        p5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.711, 8.514),

                                new Pose(11.903, 21.419)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-145))

                .build();

        p6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.903, 21.419),
                                new Pose(13.532, 18.806),
                                new Pose(10.710, 11.097)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-156), Math.toRadians(260))

                .build();

        p7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.710, 11.097),
                                new Pose(30.516, 15.823),
                                new Pose(56.710, 8.419)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(180))

                .build();

        p8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.710, 8.419),
                                new Pose(47.226, 16.081),
                                new Pose(11.548, 12.968)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();
    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        update();
        PedroDrawing.drawDebug(robot.follower);
        robot.intake.intake();
    }

    private void update(){
        switch(p){
            case 0:
                robot.turret.aim();
                halt(1);
                robot.turret.shoot3();
                followPath(p1);
                robot.turret.stopAim();
                p++;
                break;

            case 1:
                if(!robot.follower.isBusy()){
                    followPath(p2);
                    p++;
                    break;
                }

            case 2:
                if(!robot.follower.isBusy()){
                    followPath(p3);
                    p++;
                    break;
                }

            case 3:
                if(!robot.follower.isBusy()){
                    followPath(p4);
                    robot.turret.aim();
                    p++;
                    break;
                }

            case 4:
                if(!robot.follower.isBusy()){
                    halt(1);
                    robot.turret.shoot3();
                    followPath(p5);
                    robot.turret.stopAim();
                    p++;
                    break;
                }

            case 5:
                if(!robot.follower.isBusy()){
                    followPath(p6);
                    p++;
                    break;
                }

            case 6:
                if(!robot.follower.isBusy()){
                    robot.turret.aim();
                    followPath(p7);
                    p++;
                    break;
                }

            case 7:
                if(!robot.follower.isBusy()){
                    halt(1);
                    robot.turret.shoot3();
                    followPath(p8);
                    robot.turret.stopAim();
                    p++;
                    break;
                }


            default:
                break;
        }
    }
}
