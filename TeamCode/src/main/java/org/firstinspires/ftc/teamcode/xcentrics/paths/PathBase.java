package org.firstinspires.ftc.teamcode.xcentrics.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public abstract class PathBase {
    protected Pose[] P;
    protected Follower follower;
    public PathBase( Follower follower){
        this.follower = follower;
    }
    public void setPoses(Pose[] p){
        this.P = p;
    }
    public void mirrorPoints(){
        Pose[] mirroredPoints = P;
        for (int i = 0; i < P.length; i++) {
            mirroredPoints[i] = P[i].mirror();
        }
    }
}