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
        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.500, 131.600),

                                new Pose(47.500, 112.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        getFirstPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(47.500, 112.000),
                                new Pose(59.000, 80.500),
                                new Pose(18.844, 83.326)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scoreFirstPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.844, 83.326),

                                new Pose(47.500, 112.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        goToSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(47.500, 112.000),
                                new Pose(52.575, 72.233),
                                new Pose(50.603, 60.340)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        pickupSecondPattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.603, 60.340),

                                new Pose(18.542, 59.635)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        scoreSecondPattern = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.542, 59.635),
                                new Pose(49.600, 59.700),
                                new Pose(47.500, 112.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(47.500, 112.000),

                                new Pose(35.500, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))

                .build();
    }
}
