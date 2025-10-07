package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.FTCCoordinates;


import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;

import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOpControlled extends LinearOpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private CRServo intakeServo;
    private ColorSensor entranceColor;


    private Pose2D aprilTagPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private Pose ftcStandard = PoseConverter.pose2DToPose(aprilTagPose, InvertedFTCCoordinates.INSTANCE);
    private Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

    private PoseConstants poses = new PoseConstants();
    private Follower f;
    private boolean automated = false;
    private PathChain toLaunch, toPark;
    private AprilTagDetection currentDetection;
    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double speedMultiplier;

    private Timer slowDelay;

    private ColorSensed previousColor = ColorSensed.NO_COLOR;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");


        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose == null ? new Pose() : SharedData.toTeleopPose);
        f.update();

        toLaunch = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.parkPose))
                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
                .setHeadingConstraint(0)
                .build();

        initAprilTag();

        speedMultiplier = 1;
        slowDelay = new Timer();
        slowDelay.resetTimer();

        waitForStart();

        entranceColor.enableLed(true);

        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        f.startTeleopDrive(true);

        while (opModeIsActive()) {
            f.update();
            //TeleOp Drive
            if (!automated) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        false
                );
            }

            //Slow mode
            if (gamepad1.y && slowDelay.getElapsedTimeSeconds() > .5) {
                slowDelay.resetTimer();
                if (speedMultiplier == 1)
                    speedMultiplier = .1;
                else
                    speedMultiplier = 1;
            }

            //Auto Pathing to Launch
            if (gamepad1.a && !automated) {
                f.followPath(toLaunch);
                automated = true;

            }
            //Auto Pathing to Park
            else if (gamepad1.x && !automated) {
                f.followPath(toPark);
                automated = true;

            }
            //exits automated pathing
            else if ((!gamepad1.a && !gamepad1.x) && automated) {
                f.startTeleopDrive(true);
                brakeMotors();
                automated = false;
            }


            //Intake
            if (gamepad1.right_bumper)
                intakeServo.setPower(1);
            else if (gamepad1.left_bumper)
                intakeServo.setPower(-1);
            else
                intakeServo.setPower(0);


            ColorSensed currentColor = detectColor();
            if (previousColor != currentColor) {
                if (SharedData.storage[0] == ColorSensed.NO_COLOR)
                    SharedData.storage[0] = currentColor;
                else if (SharedData.storage[1] == ColorSensed.NO_COLOR)
                    SharedData.storage[1] = currentColor;
                else if (SharedData.storage[2] == ColorSensed.NO_COLOR)
                    SharedData.storage[2] = currentColor;
            }
            previousColor = currentColor;


//          telemetry.addData("Pattern", SharedData.greenIndex);
//          telemetry.addData("FOLLOWER X",f.getPose().getX());
//          telemetry.addData("FOLLOWER Y",f.getPose().getY());
//          telemetry.addData("FOLLOWER Heading",f.getPose().getHeading());
//            telemetry.addData("TAGX", currentDetection.ftcPose.x );
//            telemetry.addData("TAGY", currentDetection.ftcPose.y );
//            telemetry.addData("TAGH", currentDetection.ftcPose.yaw);
//            telemetry.addData("TAGB", currentDetection.ftcPose.bearing);
//            telemetry.addData("TAGR" , currentDetection.ftcPose.range);


            aprilTagPose = robotPose();
            telemetry.addData("RAW APRIL X", aprilTagPose.getX(DistanceUnit.INCH));
            telemetry.addData("RAW APRIL Y", aprilTagPose.getY(DistanceUnit.INCH));
            telemetry.addData("RAW APRIL H", aprilTagPose.getHeading(AngleUnit.DEGREES));


            ftcStandard = PoseConverter.pose2DToPose(aprilTagPose, InvertedFTCCoordinates.INSTANCE);
            pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            telemetry.addData("PEDRO X", pedroPose.getX());
            telemetry.addData("PEDRO Y", pedroPose.getY());
            telemetry.addData("PEDRO H", pedroPose.getPose().getHeading());


            telemetry.addData("Entrance Color", detectColor());
            telemetry.addData("hue", JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
            telemetry.addData("Storage", SharedData.storage[0] + ", " + SharedData.storage[1] + ", " + SharedData.storage[2]);
            SharedData.toTeleopPose = f.getPose();

            telemetry.update();
        }
    }

    private void brakeMotors() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void setCurr() {
        ArrayList<AprilTagDetection> detections = new ArrayList<>();

        try {
            this.currentDetection = detections.get(0);
        } catch (Exception e) {
            telemetry.addData("NO TAG DETECTED", true);
        }
    }

    public ColorSensed detectColor() {
        double hue = JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        if (hue < 180 && hue > 120)
            return ColorSensed.GREEN;
        if (hue > 200 && hue < 260)
            return ColorSensed.PURPLE;
        return ColorSensed.NO_COLOR;
    }

    public Pose2D robotPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                if (detection.metadata != null)
                    return new Pose2D(DistanceUnit.INCH, detection.ftcPose.x, detection.ftcPose.y, AngleUnit.DEGREES, detection.ftcPose.bearing);

                else
                    telemetry.addData("Metadata", "null");
            }
        }
        return null;
    }

}


