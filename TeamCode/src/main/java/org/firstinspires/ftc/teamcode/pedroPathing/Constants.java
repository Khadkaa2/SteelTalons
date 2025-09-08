package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class Constants {
    

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            .forwardZeroPowerAcceleration(-59.00706329948933)
            .lateralZeroPowerAcceleration(-95.74046809358087)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0005)

            .translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0.07, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .05, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.03, .002, 0.0032, 0.6, 0)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.69, 0, 0.008, 0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(91.8304832620753)
            .yVelocity(77.4318996233015);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.64)
            .strafePodX(-1.7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            75,
            1,
            1

    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}