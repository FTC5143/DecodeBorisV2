package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Red12paths{
    public final Pose startPose = new Pose(116.5,132,Math.toRadians(36));
    private final Pose scorePose = new Pose(84.4,83.5);
    Follower follower;
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

    public void Paths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(116.300, 131.700), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        Wait2 = 600;

        getFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(128.000, 83.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        emptyGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.000, 83.000),
                                new Pose(114.339, 79.095),
                                new Pose(126.901, 75.257)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        scoreFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(128.000, 83.000), scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        Wait6 = 600;

        goToSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(91.400, 72.200),
                                new Pose(91.900, 58.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        pickupSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.900, 58.500), new Pose(134.811, 58.507))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.811, 58.507),
                                new Pose(94.333, 59.670),
                                scorePose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Wait11 = 600;

        goToThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.213, 83.399), new Pose(99.102, 35.360))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pickUpThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(99.102, 35.360), new Pose(134.695, 35.128))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreThirdPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.695, 35.128), scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Wait14 = 600;

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(118.759, 70.023))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();
    }

    public Red12paths(Follower follower){
        this.follower = follower;
        Paths();
    }
}
