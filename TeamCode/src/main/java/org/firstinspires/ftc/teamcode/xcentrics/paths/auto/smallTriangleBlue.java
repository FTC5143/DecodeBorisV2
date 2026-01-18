package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@SuppressWarnings("ALL")
public class smallTriangleBlue {

        @SuppressWarnings("CanBeFinal")
        public PathChain goToFirstPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain getFirstPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain scoreFirstPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain goToSecondPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain getSecondPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain scoreSecondPattern;
        @SuppressWarnings("CanBeFinal")
        public PathChain park;

        @SuppressWarnings("unused")
        public smallTriangleBlue(Follower follower) {
            goToFirstPattern = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(63.500, 8.700),
                                    new Pose(56.732, 24.955),
                                    new Pose(42.292, 35.742)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            getFirstPattern = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.292, 35.742),

                                    new Pose(19.000, 35.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            scoreFirstPattern = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 35.500),

                                    new Pose(60.000, 20.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            goToSecondPattern = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 20.500),

                                    new Pose(43.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            getSecondPattern = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.000, 60.000),

                                    new Pose(19.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            scoreSecondPattern = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 60.000),

                                    new Pose(60.000, 20.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 20.500),

                                    new Pose(38.500, 33.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(90))

                    .build();
        }
    }


