package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Red9WithGatePaths {
    public PathChain scorePreload;
    public PathChain getFirstPattern;
    public PathChain emptyGate;
    public PathChain scoreFirstPattern;
    public PathChain gotoSecondPattern;
    public PathChain getSecondPattern;
    public PathChain scoreSecondPattern;
    public PathChain park;
    public Pose startPose = new Pose(116.3,131.7,Math.toRadians(36));

    public Red9WithGatePaths(Follower follower) {
        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(116.300, 131.700),

                                new Pose(84.446, 83.515)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                .build();

        getFirstPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.446, 83.515),

                                new Pose(128.000, 83.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        emptyGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(128.000, 83.000),
                                new Pose(110.363, 78.936),
                                new Pose(126.901, 75.257)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                .build();

        scoreFirstPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.901, 75.257),

                                new Pose(84.330, 83.399)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        gotoSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.330, 83.399),
                                new Pose(100.557, 75.797),
                                new Pose(91.717, 59.547)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        getSecondPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(91.717, 59.547),

                                new Pose(125.454, 58.854)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        scoreSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.454, 58.854),
                                new Pose(102.996, 69.667),
                                new Pose(84.213, 83.399)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.213, 83.399),

                                new Pose(118.759, 70.023)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))

                .build();
    }
}
