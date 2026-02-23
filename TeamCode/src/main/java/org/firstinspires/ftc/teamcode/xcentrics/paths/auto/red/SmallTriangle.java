package org.firstinspires.ftc.teamcode.xcentrics.paths.auto.red;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.xcentrics.paths.PathBase;

public class SmallTriangle extends PathBase {

    public SmallTriangle( Follower follower) {
        super( follower);
        Paths();
    }



    public PathChain gotoscorepose;
    public PathChain turntogetfirstpattern;
    public PathChain pickupfirstpattern;
    public PathChain scorefirstpattern;
    public PathChain gotosecondpattern;
    public PathChain pickupsecondpattern;
    public PathChain scoresecondpattern;
    public PathChain park;

    private void Paths() {

        gotoscorepose = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[0],
                                P[1]
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        turntogetfirstpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[1],
                                P[2]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        pickupfirstpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[2],
                                P[3]
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorefirstpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[3],
                                P[1]
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        gotosecondpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[1],
                                P[4]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        pickupsecondpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[4],
                                P[5]
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        scoresecondpattern = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[5],
                                P[1]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                P[1],
                                P[6]
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

}
