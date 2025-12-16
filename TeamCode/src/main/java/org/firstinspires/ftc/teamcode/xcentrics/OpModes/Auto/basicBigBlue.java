package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        private final PathChain scorePreload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build()
                ,travelToPattern = robot.follower.pathBuilder()
                .addPath(new BezierLine(scorePose,travlePose))
                .setConstantHeadingInterpolation(travlePose.getHeading())
                .addPath(new BezierLine(travlePose,patternPose))
                .setConstantHeadingInterpolation(patternPose.getHeading())
                .build()
                , scorePattern = robot.follower.pathBuilder()
                .addPath(new BezierLine(patternPose,scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build()
                ,park = robot.follower.pathBuilder()
                .addPath(new BezierLine(scorePose,parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
        private int pathState = 0;
        @Override
        public void on_init() {

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
                    robot.follower.followPath(scorePreload);
                    pathState++;
                    break;

                case 1:
                    if(!robot.follower.isBusy()){
                        robot.intake.setPower(1);
                        robot.turret.launch();
                        robot.intake.setPower(0);

                        halt(0.5);

                        robot.intake.setPower(1);
                        halt(0.5);
                        robot.intake.setPower(0);
                        robot.turret.launch();

                        halt(0.5);

                        robot.intake.setPower(1);
                        halt(0.5);
                        robot.intake.setPower(0);
                        robot.turret.launch();

                        halt(0.5);

                        pathState++;
                        break;
                    }

                case 2:
                    robot.follower.followPath(travelToPattern);
                    robot.intake.setPower(1);
                    pathState++;
                    break;

                case 3:
                    if(!robot.follower.isBusy()){
                        robot.follower.followPath(scorePattern);
                        pathState++;
                        break;
                    }

                case 4:
                    if(!robot.follower.isBusy()){
                        robot.intake.setPower(1);
                        robot.turret.launch();
                        robot.intake.setPower(0);

                        halt(0.5);

                        robot.intake.setPower(1);
                        halt(0.5);
                        robot.intake.setPower(0);
                        robot.turret.launch();

                        halt(0.5);

                        robot.intake.setPower(1);
                        halt(0.5);
                        robot.intake.setPower(0);
                        robot.turret.launch();

                        halt(0.5);

                        pathState++;
                        break;
                    }

                case 5:
                    robot.follower.followPath(park);
                    pathState++;
                    break;
            }
        }
    }
