package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Spin;

public class big12blue extends LiveAutoBase {
    public static double wait = 2;
    private final Pose startPose = new Pose(27.5,131.6,r(144)),
                       scorePose1 = new Pose(58.5,85,r(90)),
                       scorePose = new Pose(58.5,85,r(180));
    private final PathChain scorePreload = robot.follower.pathBuilder()
            .addPath(new BezierLine(startPose,scorePose1))
            .setLinearHeadingInterpolation(startPose.getHeading(),scorePose1.getHeading())
            .build();
    private final PathChain getFirstPattern = robot.follower.pathBuilder()
            .addPath(new BezierLine(scorePose1,new Pose(44,85)))
            .setLinearHeadingInterpolation(scorePose1.getHeading(),180)
            .addParametricCallback(0.5,robot.intake.intake())
            .addPath(new BezierLine(new Pose(44,85),new Pose(15,85)))
            .setConstantHeadingInterpolation(180)
            .build();
    private final PathChain scoreFirstPattern = robot.follower.pathBuilder()
            .addPath(new BezierLine(new Pose(15,85),scorePose))
            .setConstantHeadingInterpolation(180)
            .addParametricCallback(0,robot.intake.stopIntake())
            .addPoseCallback(new Pose(43,85),robot.turret.aim(), 0.6)
            .build();
    private final PathChain getSecondPattern = robot.follower.pathBuilder()
            .addPath(new BezierCurve(scorePose,
                    new Pose(56.600, 50.800),
                    new Pose(5.902, 60.315),
                    new Pose(37.200, 59.100),
                    new Pose(9.300, 59.000)
            ))
            .setConstantHeadingInterpolation(180)
            .addParametricCallback(0,robot.turret.stopAim())
            .addPoseCallback(new Pose(47.5,65.4),robot.intake.intake(),0.4)
            .build();
    private final PathChain emptyGate = robot.follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(9.300, 59.000),
                            new Pose(13.500, 52.900),
                            new Pose(34.230, 63.266),
                            new Pose(16.500, 70.000)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
            .addParametricCallback(0.1,robot.intake.stopIntake())
            .build();
    private final PathChain scoreSecondPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(new Pose(16.500, 70.000), new Pose(58.500, 85.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
            .addParametricCallback(0.5,robot.turret.aim())
            .build();

    private final PathChain getThirdPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            new Pose(58.500, 85.000),
                            new Pose(65.700, 26.000),
                            new Pose(36.000, 44.600),
                            new Pose(67.000, 31.000),
                            new Pose(9.000, 36.000)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0,robot.turret.stopAim())
            .addPoseCallback(new Pose(49,37.2),robot.intake.intake(), 0.6)
            .build();
    private final PathChain scoreThirdPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(new Pose(9.000, 36.000), new Pose(58.800, 85.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0,robot.intake.stopIntake())
            .addParametricCallback(0.5,robot.turret.aim())
            .build();
    private final PathChain park = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(new Pose(58.800, 85.000), new Pose(47.000, 76.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addParametricCallback(0,robot.turret.stopAim())
            .build();
    private static int pathState = 0;
    @Override
    public void on_init() {
        robot.follower.setStartingPose(startPose);
        robot.spin.setBalls(Spin.Ball.PURPLE, Spin.Ball.PURPLE,Spin.Ball.GREEN);
    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {

    }
    private double r(double d){
        return Math.toRadians(d);
    }
    private void autoPathUpdate(){
        switch (pathState){
            case 0:
                robot.follower.followPath(scorePreload);
                pathState++;
                break;

            case 1:
                if(!robot.follower.isBusy()){
                    if(robot.getPattern() != null){
                        robot.spin.ejectPattern();
                        halt(wait);
                        robot.follower.followPath(getFirstPattern);
                        pathState++;
                        break;
                    }
                }

            case 2:
                if(!robot.follower.isBusy()){
                    robot.follower.followPath(scoreFirstPattern);
                    pathState++;
                    break;
                }

            case 3:
                if(!robot.follower.isBusy()){
                    robot.spin.ejectPattern();
                    halt(wait);
                    robot.follower.followPath(getSecondPattern);
                    pathState++;
                    break;
                }

            case 4:
                if(!robot.follower.isBusy()){
                    robot.follower.followPath(emptyGate);
                    pathState++;
                    break;
                }

            case 5:
                if(!robot.follower.isBusy()){
                    robot.follower.followPath(scoreSecondPattern);
                    pathState++;
                    break;
                }

            case 6:
                if(!robot.follower.isBusy()){
                    robot.spin.ejectPattern();
                    halt(wait);
                    robot.follower.followPath(getThirdPattern);
                    pathState++;
                    break;
                }

            case 7:
                if(!robot.follower.isBusy()){
                    robot.follower.followPath(scoreThirdPattern);
                    pathState++;
                    break;
                }

            case 8:
                if(!robot.follower.isBusy()){
                    robot.spin.ejectPattern();
                    halt(wait);
                    robot.follower.followPath(park);
                    pathState++;
                    break;
                }

            case 9:
                if(!robot.follower.isBusy()){
                    robot.addData("AutoDone:",true);
                    robot.setLastPose(robot.follower.getPose());
                    break;
                }
        }
    }
}
