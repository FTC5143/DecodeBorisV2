package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.red;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.components.live.Spin;

public class big12red extends LiveAutoBase {
    public static double wait = 2;
    //line endpoints
    private final Pose startPose = new Pose(116.5,132,r(36)),
            scorePose1 = new Pose(83,100,r(0)),
            rotatePose = new Pose(44,85,r(0)),
            pickup1 = new Pose(15,85,r(0)),
            scorePose = new Pose(58.5,85,r(0)),
            pickup2 = new Pose(9.3,59,r(0)),
            pickup2pt2 = new Pose(135,59,r(0)),
            emptyGatePose = new Pose(16.5,70,r(90)),
            pickup3 = new Pose(9,36,r(0)),
            parkPose = new Pose(47,76,r(0));

    //control points for lines
    private final Pose p21 = new Pose(56.6,50.8),
            p22 = new Pose(6,60.3),
            g1 = new Pose(13.5,52.9),
            g2 = new Pose(34.2,63.3),
            p31 = new Pose(65.7,26),
            p32 = new Pose(36,44.6),
            p33 = new Pose(67,31);

    private final PathChain scorePreload = robot.follower.pathBuilder()
            .addPath(new BezierLine(startPose,scorePose1))
            .setLinearHeadingInterpolation(startPose.getHeading(),scorePose1.getHeading())
            .build();
    private final PathChain getFirstPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(scorePose1, rotatePose))
            .setLinearHeadingInterpolation(scorePose1.getHeading(),rotatePose.getHeading())

            .addParametricCallback(0.5,robot.intake.intake())

            .addPath(
                    new BezierLine(rotatePose,pickup1)
            )
            .setConstantHeadingInterpolation(180)
            .build();
    private final PathChain scoreFirstPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(pickup1,scorePose)
            )
            .setConstantHeadingInterpolation(180)

            .addParametricCallback(0,robot.intake.stopIntake())
            .addPoseCallback(new Pose(43,85),robot.turret.aim(), 0.6)

            .build();
   /* private final PathChain getSecondPattern = robot.follower.pathBuilder()
            .addPath(new BezierCurve(scorePose,
                    p21,
                    p22,
                    pickup2
            ))
            .setConstantHeadingInterpolation(180)

            .addParametricCallback(0,robot.turret.stopAim())
            .addParametricCallback(1,robot.intake.intake())

            .addPath(
                    new BezierLine()
            )

            .build(); */
    private final PathChain emptyGate = robot.follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            pickup2,
                            g1,
                            g2,
                            emptyGatePose
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

            .addParametricCallback(0.1,robot.intake.stopIntake())

            .build();
    private final PathChain scoreSecondPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(emptyGatePose,scorePose)
            )
            .setLinearHeadingInterpolation(emptyGatePose.getHeading(), scorePose.getHeading())

            .addParametricCallback(0.5,robot.turret.aim())

            .build();

    private final PathChain getThirdPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierCurve(
                            scorePose,
                            p31,
                            p32,
                            p33,
                            pickup3
                    )
            )
            .setConstantHeadingInterpolation(pickup3.getHeading())

            .addParametricCallback(0,robot.turret.stopAim())
            .addPoseCallback(new Pose(49,37.2),robot.intake.intake(), 0.6)

            .build();
    private final PathChain scoreThirdPattern = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(pickup3,scorePose)
            )
            .setConstantHeadingInterpolation(pickup3.getHeading())

            .addParametricCallback(0,robot.intake.stopIntake())
            .addParametricCallback(0.5,robot.turret.aim())

            .build();
    private final PathChain park = robot.follower.pathBuilder()
            .addPath(
                    new BezierLine(scorePose,parkPose)
            )
            .setConstantHeadingInterpolation(parkPose.getHeading())
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
        autoPathUpdate();
        robot.addData("PathState",pathState);
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
                        robot.spin.ejectPPG();
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
                   // robot.follower.followPath(getSecondPattern);
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
