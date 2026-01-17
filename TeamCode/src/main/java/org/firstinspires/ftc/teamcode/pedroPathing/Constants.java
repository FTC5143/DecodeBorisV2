package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.8)
            .forwardZeroPowerAcceleration(-48.526847126531436)
            .lateralZeroPowerAcceleration(-86.03851261899878)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.1,0.5,0,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0019573536701847456)
            .strafeTicksToInches(0.0019753046721908427)
            .turnTicksToInches(0.0019305918124591602)
            .leftPodY(6.5)
            .rightPodY(-6.5)
            .strafePodX(-6)
            .leftEncoder_HardwareMapName("FR")// port 0
            .rightEncoder_HardwareMapName("BL") //port 3
            .strafeEncoder_HardwareMapName("BR") //port 1
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(54.81069716380982)
            .yVelocity(36.28748043764696);
}