package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.8);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.004634574217659694)
            .strafeTicksToInches(0.001954888478995178)
            .turnTicksToInches(0.001971553435306175)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-7)
            .leftEncoder_HardwareMapName("FR")
            .rightEncoder_HardwareMapName("BL")
            .strafeEncoder_HardwareMapName("BR")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
}
