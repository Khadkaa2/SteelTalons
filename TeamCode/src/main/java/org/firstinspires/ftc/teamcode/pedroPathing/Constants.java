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
            .forwardZeroPowerAcceleration(-35.4126335839)
            .lateralZeroPowerAcceleration(-49.616811183)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            //.centripetalScaling(0.0005)

            .translationalPIDFCoefficients(new PIDFCoefficients(.5, 0, .075, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.015, 0, 0, 0.6, 0)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .secondaryDrivePIDFCoefficients( new FilteredPIDFCoefficients(0,0,0,0,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(67.0965584823)
            .yVelocity(57.9228971226);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.30709)
            .strafePodX(-1.88976)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .yawScalar(1.0)
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