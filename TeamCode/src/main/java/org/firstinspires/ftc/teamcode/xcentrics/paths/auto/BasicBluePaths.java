package org.firstinspires.ftc.teamcode.xcentrics.paths.auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.xcentrics.paths.PathBase;

public class BasicBluePaths extends PathBase {
    Pose startPose = new Pose(28.2,132,r(145)), scorePose = new Pose(45,97);
    PathChain scorePreload, getFirstPattern, scoreFirstPattern;
    PathChain getSecondPattern, scoreSecondPattern, park;
    private double r(double r){
        return Math.toRadians(r);
    }
    @Override
    public void buildPaths(int pattern) {
        //1 = gpp, 2 = pgp, 3 = ppg

    }

    @Override
    public PathChain scorePreload() {
        return null;
    }

    @Override
    public PathChain getFirstPattern() {
        return null;
    }

    @Override
    public PathChain scoreFirstPattern() {
        return null;
    }

    @Override
    public PathChain getSecondPattern() {
        return null;
    }

    @Override
    public PathChain park() {
        return null;
    }
}
