package org.firstinspires.ftc.teamcode.xcentrics.paths.auto.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarBlueAutoPaths {
    public PathChain p1;
    public PathChain p2;
    public PathChain p3;
    public PathChain p4;
    public PathChain p5;
    public PathChain p6;
    public PathChain p7;
    public PathChain p8;

    public FarBlueAutoPaths(Follower follower) {
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
}
