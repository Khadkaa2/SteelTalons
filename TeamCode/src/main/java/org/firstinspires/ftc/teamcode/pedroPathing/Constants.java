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
            .forwardZeroPowerAcceleration(-31.7995657045)
            .lateralZeroPowerAcceleration(-71.9777507807)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            //.centripetalScaling(0.0005)

            .translationalPIDFCoefficients(new PIDFCoefficients(1, .001, .15, .04))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .05, .04))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(.05, 0, .0001, .6, .035)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, .025))
            .secondaryDrivePIDFCoefficients( new FilteredPIDFCoefficients(.01,0,0,0,.035))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.15,0,0.01,0.015));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.7441753237)
            .yVelocity(50.6550690899);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.30709)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .yawScalar(1.0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            .95,
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