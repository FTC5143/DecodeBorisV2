package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue12paths {
    public Pose startPose = new Pose(27.5,131.6,Math.toRadians(144));
    public PathChain scorePreload;
    public double Wait2;
    public PathChain getFirstPattern;
    public PathChain emptyGate;
    public PathChain scoreFirstPattern;
    public double Wait6;
    public PathChain goToSecondPattern;
    public PathChain pickupSecondPattern;
    public PathChain scoreSecondPattern;
    public double Wait11;
    public PathChain goToThirdPattern;
    public PathChain pickUpThirdPattern;
    public PathChain scoreThirdPattern;
    public double Wait14;
    public PathChain park;

    public Blue12paths(Follower follower) {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.500, 131.600), new Pose(59.554, 83.515))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Wait2 = 600;

        getFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.554, 83.515), new Pose(17.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        emptyGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.000, 83.000),
                                new Pose(29.661, 79.095),
                                new Pose(17.099, 75.257)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        scoreFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(17.099, 75.257), new Pose(59.670, 83.399))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Wait6 = 600;

        goToSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.670, 83.399),
                                new Pose(52.575, 72.233),
                                new Pose(52.110, 58.507)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        pickupSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(52.110, 58.507), new Pose(9.189, 58.507))
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.189, 58.507),
                                new Pose(49.667, 59.670),
                                new Pose(59.787, 83.399)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Wait11 = 600;

        goToThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.787, 83.399), new Pose(44.898, 35.360))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickUpThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.898, 35.360), new Pose(9.305, 35.128))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scoreThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(9.305, 35.128), new Pose(59.670, 83.515))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Wait14 = 600;

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.670, 83.515), new Pose(25.241, 70.023))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }
}
