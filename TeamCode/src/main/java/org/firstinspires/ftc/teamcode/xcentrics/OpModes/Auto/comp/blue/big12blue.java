//package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//
//import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
//
//@Configurable
//public class big12blue extends LiveAutoBase {
//    public static double wait = 2;
//    public static double launchWait = 0.3;
//    private final Pose startPose = new Pose(27.5,131.6,r(144));
//
//    //line endpoints
//    private final Pose scorePose = new Pose(59.5,83.5),
//                              pickup1Pose = new Pose(16,83,r(180)),
//                              emptyGatePose = new Pose(17,75.3,r(90)),
//                              goToSecondPatternPose = new Pose(52.1,58.6,r(180)),
//                              pickup2ndPatternPose = new Pose(59.8,83.4,r(180)),
//                              goToThirdPatternPose = new Pose(44.9,35.4,r(180)),
//                              pickup3rdPatternPose = new Pose(9.3,35.1,r(180)),
//                              parkAtGatePose = new Pose(25.3,70,r(270));
//    //points
//    private final Pose e1 = new Pose(29.7,79), s1 = new Pose(52.6,72.2),
//    s2 = new Pose(49.7,59.7);
//
//    //paths
//    private final PathChain scorePreload = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierLine(startPose,scorePose)
//            )
//            .setLinearHeadingInterpolation(startPose.getHeading(),r(180))
//            .addParametricCallback(0,robot.turret.aim())
//            .build();
//
//    private final PathChain getFirstPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierLine(scorePose,pickup1Pose)
//            )
//            .setConstantHeadingInterpolation(pickup1Pose.getHeading())
//            .addParametricCallback(0,robot.turret.stopAim())
//            .addParametricCallback(0,robot.intake.intake())
//            .build();
//
//    private final PathChain emptyGate = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierCurve(pickup1Pose,e1,emptyGatePose)
//            )
//            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), emptyGatePose.getHeading())
//            .addParametricCallback(0.5,robot.intake.stopIntake())
//            .build();
//
//    private final PathChain scoreFirstPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierLine(emptyGatePose,scorePose)
//            )
//            .setConstantHeadingInterpolation(emptyGatePose.getHeading())
//            .addParametricCallback(0.5,robot.turret.aim())
//            .build();
//
//    private final PathChain getSecondPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierCurve(scorePose,s1,goToSecondPatternPose)
//            )
//            .setLinearHeadingInterpolation(scorePose.getHeading(), goToSecondPatternPose.getHeading())
//            .build();
//    private final PathChain goToSecondPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierLine(goToSecondPatternPose,pickup2ndPatternPose)
//            )
//            .setConstantHeadingInterpolation(goToSecondPatternPose.getHeading())
//            .addParametricCallback(0,robot.intake.intake())
//            .addParametricCallback(0,setMaxPower(0.5))
//            .build();
//    private final PathChain scoreSecondPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierCurve(pickup2ndPatternPose,scorePose)
//            )
//            .setConstantHeadingInterpolation(pickup2ndPatternPose.getHeading())
//            .addParametricCallback(0.2,robot.intake.stopIntake())
//            .build();
//    private final PathChain getThirdPattern = robot.follower.pathBuilder()
//            .addPath(
//                    new BezierLine(scorePose,goToThirdPatternPose)
//            )
//
//    private static int pathState = 0;
//
//    @Override
//    public void on_init() {
//        robot.follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public void on_start() {
//
//    }
//
//    @Override
//    public void on_stop() {
//
//    }
//
//    @Override
//    public void on_loop() {
//        autoPathUpdate();
//        robot.addData("PathState",pathState);
//    }
//    private double r(double d){
//        return Math.toRadians(d);
//    }
//    private void autoPathUpdate(){
//        switch (pathState){
//            case 0:
//                robot.follower.followPath(scorePreload);
//                pathState++;
//                break;
//
//            case 1:
//                if(!robot.follower.isBusy()){
//                    if(robot.getPattern() != null){
//                        shoot3();
//                        robot.follower.followPath(getFirstPattern);
//                        pathState++;
//                        break;
//                    }
//                }
//
//            case 2:
//                if(!robot.follower.isBusy()){
//                    robot.follower.followPath(emptyGate);
//                    pathState++;
//                    break;
//                }
//
//            case 3:
//                if(!robot.follower.isBusy()){;
//                    robot.follower.followPath(scoreFirstPattern);
//                    pathState++;
//                    break;
//                }
//
//            case 4:
//                if(!robot.follower.isBusy()){
//                    shoot3();
//                    robot.follower.followPath(getSecondPattern);
//                    pathState++;
//                    break;
//                }
//
//            case 5:
//                if(!robot.follower.isBusy()){
//                    robot.follower.followPath(scoreSecondPattern);
//                    pathState++;
//                    break;
//                }
//
//            case 6:
//                if(!robot.follower.isBusy()){
//                    shoot3();
//                    robot.follower.followPath(getThirdPattern);
//                    pathState++;
//                    break;
//                }
//
//            case 7:
//                if(!robot.follower.isBusy()){
//                    robot.follower.followPath(scoreThirdPattern);
//                    pathState++;
//                    break;
//                }
//
//            case 8:
//                if(!robot.follower.isBusy()){
//                    shoot3();
//                    robot.follower.followPath(park);
//                    pathState++;
//                    break;
//                }
//
//            case 9:
//                if(!robot.follower.isBusy()){
//                    robot.addData("AutoDone:",true);
//                    robot.setLastPose(robot.follower.getPose());
//                    break;
//                }
//        }
//
//    }
//
//    private Runnable setMaxPower(double maxPower){
//        return () -> robot.follower.setMaxPower(maxPower);
//    }
//    private void shoot3(){
//        // 1
//        robot.turret.launch();
//        halt(launchWait);
//        //get next one ready
//        robot.intake.intake().run();
//        halt(launchWait);
//        //launch 2
//        robot.intake.stopIntake().run();
//        robot.turret.launch();
//        halt(launchWait);
//        //get next one ready
//        robot.intake.intake().run();
//        halt(launchWait);
//
//        //launch third one
//        robot.intake.stopIntake().run();
//        robot.turret.launch();
//        halt(launchWait);
//    }
//}
