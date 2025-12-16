package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "basicBigBlue")
public class basicBigBlue extends LiveAutoBase{
        private final Pose startPose = new Pose(116.6,131.7,r(37)),
                scorePose = new Pose(96,96,r(0)),
                travlePose = new Pose(96,84,r(0)),
                patternPose = new Pose(129,84,r(0)),
                parkPose = new Pose(100,75,r(0));

        private double r(double r){
            return Math.toRadians(r);
        }
        private Follower follower = Constants.createFollower(hardwareMap);
        private final PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build()
                ,travelToPattern = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,travlePose))
                .setConstantHeadingInterpolation(travlePose.getHeading())
                .addPath(new BezierLine(travlePose,patternPose))
                .setConstantHeadingInterpolation(patternPose.getHeading())
                .build()
                , scorePattern = follower.pathBuilder()
                .addPath(new BezierLine(patternPose,scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build()
                ,park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
        private int pathState = 0;
        @Override
        public void on_init() {
            robot.isRed = false;
        }

        @Override
        public void on_start() {

        }

        @Override
        public void on_stop() {

        }

        @Override
        public void on_loop() {
            autoPathUpdate();
            robot.addData("PathState",pathState);
            robot.follower.setMaxPower(0.5);
        }

        public void autoPathUpdate(){
            switch (pathState){
                case 0:
                    follower.followPath(scorePreload);
                    pathState++;
                    break;

                case 1:
                    if(!follower.isBusy()){
                        shoot1ball();
                        shoot1ball();
                        shoot1ball();

                        pathState++;
                        break;
                    }

                case 2:
                    follower.followPath(travelToPattern);
                    robot.intake.setPower(1);
                    pathState++;
                    break;

                case 3:
                    if(!follower.isBusy()){
                        follower.followPath(scorePattern);
                        pathState++;
                        break;
                    }

                case 4:
                    if(!follower.isBusy()){
                        shoot1ball();
                        shoot1ball();
                        shoot1ball();

                        pathState++;
                        break;
                    }

                case 5:
                    follower.followPath(park);
                    pathState++;
                    break;
            }
        }
        public void shoot1ball(){
            robot.intake.setPower(1);
            robot.turret.launch();
            robot.intake.setPower(0);
            halt(0.5);
        }
    }
