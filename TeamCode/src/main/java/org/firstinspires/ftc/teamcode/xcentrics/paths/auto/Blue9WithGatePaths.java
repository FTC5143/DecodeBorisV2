package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue9WithGatePaths {
    public PathChain scorePreload;
    public PathChain getFirstPattern;
    public PathChain emptyGate;
    public PathChain scoreFirstPattern;
    public PathChain goToSecondPattern;
    public PathChain getSecondPattern;
    public PathChain scoreSecondPattern;
    public PathChain park;
    public Pose startPose = new Pose(27.5,131.6,Math.toRadians(144));

    public Blue9WithGatePaths(Follower follower) {
        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.500, 131.600),

                                new Pose(59.554, 83.515)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        getFirstPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.554, 83.515),

                                new Pose(16.000, 83.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        emptyGate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(16.000, 83.000),
                                new Pose(33.571, 81.068),
                                new Pose(17.099, 75.257)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        scoreFirstPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.099, 75.257),

                                new Pose(59.670, 83.399)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        goToSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.670, 83.399),
                                new Pose(52.575, 72.233),
                                new Pose(52.110, 58.507)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        getSecondPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(52.110, 58.507),

                                new Pose(9.189, 58.507)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.189, 58.507),
                                new Pose(39.715, 76.565),
                                new Pose(59.787, 83.399)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.787, 83.399),

                                new Pose(25.241, 70.023)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();
    }
}
