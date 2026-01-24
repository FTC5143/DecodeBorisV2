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
                        new BezierLine(new Pose(116.500, 131.600), new Pose(96.500, 112.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        getFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.500, 112.000),
                                new Pose(85.000, 80.500),
                                new Pose(127.500, 85.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreFirstPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.500, 85.000), new Pose(96.500, 112.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        goToSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.500, 112.000),
                                new Pose(91.425, 72.233),
                                new Pose(91.890, 59.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pickupSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.890, 59.000), new Pose(134.500, 59.300))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scoreSecondPattern = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.500, 59.300),
                                new Pose(94.400, 59.700),
                                new Pose(96.500, 112.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 112.000), new Pose(107, 77))

                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    
    public Red12paths(Follower follower){
        this.follower = follower;
        Paths();
    }
}
