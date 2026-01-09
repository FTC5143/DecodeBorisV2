package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Blue12paths {
    public Pose startPose = new Pose(27.5,131.6,Math.toRadians(144));
    public Pose scorePose = new Pose(56,95);
    public PathChain scorePreload;
    public PathChain getFirstPattern;
    public PathChain scoreFirstPattern;
    public PathChain goToSecondPattern;
    public PathChain pickupSecondPattern;
    public PathChain scoreSecondPattern;
    public PathChain park;

    public Blue12paths(Follower follower) {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.500, 131.600), scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        getFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(16.500, 85.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scoreFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.000, 83.000), scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(52.575, 72.233),
                                new Pose(52.110, 58.507)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickupSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(52.110, 58.507), new Pose(9.5, 59))
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.189, 58.507),
                                new Pose(49.667, 59.670),
                                scorePose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose, new Pose(25.241, 70.023))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }
}
