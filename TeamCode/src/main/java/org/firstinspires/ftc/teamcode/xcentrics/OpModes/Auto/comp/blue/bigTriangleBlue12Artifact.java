package org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.comp.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.xcentrics.OpModes.Auto.LiveAutoBase;
import org.firstinspires.ftc.teamcode.xcentrics.paths.auto.Blue12paths;
import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;

@Configurable
@Autonomous(name = "Blue Big Triangle (12 Artifacts)")
public class bigTriangleBlue12Artifact extends LiveAutoBase {
    private Blue12paths paths;
    public  int pathState = 1;
    public static boolean b1;
    public static double launchWait = 0.7;
    public static double maxSpeed = 1.5;
    public static double intakeSpeed = 0.5;
    @Override
    public void on_init() {
        Robot.isRed = false;
        paths = new Blue12paths(robot.follower);
        robot.follower.setStartingPose(paths.startPose);
    }

    @Override
    public void on_start() {

    }

    @Override
    public void on_stop() {

    }

    @Override
    public void on_loop() {
        update();
        robot.turret.update(this);
        drawOnlyCurrent();
    }

    private void update(){
        switch(pathState){
            case 1:
                //follow score preload path
                followPath(paths.scorePreload);
                //aim the turret
                robot.turret.aim().run();
                //increment
                pathState++;
                break;

            case 2:
                if(notBusy()){
                    //score preload
                    halt(1);
                        shoot3();
                        //run intake
                        robot.intake.intake().run();
                        //stop aiming the turret
                        robot.turret.stopAim().run();
                        //follow pickup path slowly
                        followPath(paths.getFirstPattern, intakeSpeed);
                        //increment
                        pathState = 4;
                    break;
                }

            case 3:
                if(notBusy()){
                    //stop intake
                    robot.intake.stopIntake().run();
                    //empty gate
                    followPath(paths.emptyGate);
                    //increment
                    pathState++;
                    break;
                }

            case 4:
                if(notBusy()){
                    //aim turret
                    robot.turret.aim().run();
                    //go to score position at full speed
                    followPath(paths.scoreFirstPattern,maxSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 5:
                if(notBusy()) {
                    //score first pattern
                    shoot3();
                    //stop aiming
                    robot.turret.stopAim().run();

                    //go to second pattern
                    followPath(paths.goToSecondPattern);
                    //increment
                    pathState++;
                    break;
                }

            case 6:
                if(notBusy()){
                    //turn on intake
                    robot.intake.intake().run();
                    //pickup second pattern slowly
                    followPath(paths.pickupSecondPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 7:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake().run();
                    //go to score position at full speed
                    followPath(paths.scoreSecondPattern,maxSpeed);
                    //aim turret
                    robot.turret.aim().run();
                    //increment
                    pathState = 11;
                    break;
                }
            case 8:
                if(notBusy()){
                    //score second pattern
                    shoot3();
                    //stop aiming the turret
                    robot.turret.stopAim();

                    //go to the third pattern
                    followPath(paths.goToThirdPattern);
                    //increment
                    pathState++;
                    break;
                }

            case 9:
                if(notBusy()){
                    //turn on intake
                    robot.intake.intake().run();
                    //pick up balls slowly
                    followPath(paths.pickUpThirdPattern,intakeSpeed);
                    //increment
                    pathState++;
                    break;
                }

            case 10:
                if(notBusy()){
                    //turn off intake
                    robot.intake.stopIntake().run();
                    //go to scoring position at full speed
                    followPath(paths.scoreThirdPattern,maxSpeed);
                    //aim the turret
                    robot.turret.aim().run();
                    //increment
                    pathState++;
                    break;
                }

            case 11:
                if(notBusy()){
                    //score third pattern
                    shoot3();
                    //stop aiming the turret
                    robot.turret.stopAim().run();

                    //go to the park position
                    followPath(paths.park,1);
                    pathState = 500;
                }
        }
    }
    private boolean notBusy(){
        return !robot.follower.isBusy();
    }
    private void shoot3(){
        // 1
        halt(0.5);
        waitUntillSpin();
        robot.turret.launch();
        robot.update();
        //get next one ready
        robot.intake.setPower(-1);
        robot.update();
        halt(launchWait);
        //launch 2
        waitUntillSpin();
        robot.turret.launch();
        robot.update();

        //get next one ready
        halt(1.5);

        //launch third one

        robot.update();
        waitUntillSpin();
        robot.turret.launch();
        robot.intake.setPower(0);
        robot.update();
    }
    public void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(robot.follower.getPose());
            Drawing.drawPath(robot.follower.getCurrentPath(),new Style("","#3F51B5",0.75));
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
    private void waitUntillSpin(){
        while((Math.abs(robot.turret.fly1.getVelocity() - 1600) <= 0) ){
            robot.update();
        }
    }
}
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}