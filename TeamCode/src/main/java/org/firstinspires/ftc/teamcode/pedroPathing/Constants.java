package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.88412)
            .forwardZeroPowerAcceleration(-25.265)
            .lateralZeroPowerAcceleration(-78.167)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.005, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0002, 0.3, 0.01))
            .centripetalScaling(0.0004);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.986)
            .yVelocity(62.715);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .turnTicksToInches(0.003229438)
            .leftPodY(6.7)
            .rightPodY(-6.7)
            .strafePodX(-5)
            .leftEncoder_HardwareMapName("leftFrontDrive")
            .rightEncoder_HardwareMapName("rightFrontDrive")
            .strafeEncoder_HardwareMapName("rightBackDrive")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .strafeTicksToInches(00.002943746)
            .forwardTicksToInches(0.0029757745);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.35, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)


                .build();
    }
}
