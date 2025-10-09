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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
    private DcMotorEx sortMotor = null;
    private CRServo intakeServo;
    private ColorSensor entranceColor;

    private PoseConstants poses = new PoseConstants();
    private Follower f;
    private boolean automated = false;
    private PathChain toLaunch, toPark;
    private AprilTagDetection currentDetection;
    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private Orientation rot;

    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double speedMultiplier;

    private Timer slowDelay, colorTimer, launchTimer;

    private ColorSensed previousColor = ColorSensed.NO_COLOR;

    public void runOpMode() throws InterruptedException {

        //init hardware
        sortMotor = hardwareMap.get(DcMotorEx.class, "sortMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");

        //init sortMotor
        sortMotor.setTargetPosition(sortMotor.getCurrentPosition());
        sortMotor.setTargetPositionTolerance(10);
        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortMotor.setPower(1);

        //init follower
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose == null ? poses.START_POSE : SharedData.toTeleopPose);
        f.update();

        //create paths
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

        //init timers/constants
        speedMultiplier = 1;
        slowDelay = new Timer();
        colorTimer = new Timer();
        launchTimer = new Timer();
        colorTimer.resetTimer();
        slowDelay.resetTimer();
        launchTimer.resetTimer();

        waitForStart();

        entranceColor.enableLed(true);

        f.startTeleopDrive(true);

        while (opModeIsActive()) {
            f.update();
            //TeleOp Drive (field oriented)
            if (!automated) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        false
                );
            }

            //Slow mode (.1 speed)
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
                automated = false;
            }


            //Intake powers
            //may want to set it so that it is always slightly intakes if necessary later
            if (gamepad1.right_bumper)
                intakeServo.setPower(1);
            else if (gamepad1.left_bumper)
                intakeServo.setPower(-1);
            else
                intakeServo.setPower(0);

            //Checks if it has been enough time since last launch
            //dpad up for green
            //dpad down for purple
            //dpad left to clear storage (for testing)
            if(launchTimer.getElapsedTimeSeconds()>1){
                //green
                if (gamepad1.dpad_up) {
                    launchTimer.resetTimer();
                    int ind = -1;
                    if (SharedData.storage[0] == ColorSensed.GREEN)
                        ind = 0;
                    else if (SharedData.storage[1] == ColorSensed.GREEN)
                        ind = 1;
                    else if (SharedData.storage[2] == ColorSensed.GREEN)
                        ind = 2;
                    if (ind != -1) {
                        setStoragePos(ind, false);
                        SharedData.storage[ind] = ColorSensed.NO_COLOR;
                    }
                }
                //purple
                else if (gamepad1.dpad_down) {
                    launchTimer.resetTimer();
                    int ind = -1;
                    if (SharedData.storage[0] == ColorSensed.PURPLE)
                        ind = 0;
                    else if (SharedData.storage[1] == ColorSensed.PURPLE)
                        ind = 1;
                    else if (SharedData.storage[2] == ColorSensed.PURPLE)
                        ind = 2;
                    if (ind != -1) {
                        setStoragePos(ind, false);
                        SharedData.storage[ind] = ColorSensed.NO_COLOR;
                    }
                }
                else if (gamepad1.dpad_left){
                    SharedData.emptyStorage();
                }
            }

            //Senses if a ball is in the intake area
            //Sets sorting position to open area on detection
            //need to figure out a way to swap off of the slot to open slot after intake rather that directly before
            //-> distance sensor -> check if ball is in area after sensing stops
            ColorSensed currentColor = detectColor();
            if (previousColor != currentColor && colorTimer.getElapsedTimeSeconds() > .5) {
                colorTimer.resetTimer();
                if (SharedData.storage[0] == ColorSensed.NO_COLOR && currentColor != ColorSensed.NO_COLOR) {
                    SharedData.storage[0] = currentColor;
                    setStoragePos(0, true);
                } else if (SharedData.storage[1] == ColorSensed.NO_COLOR && currentColor != ColorSensed.NO_COLOR) {
                    SharedData.storage[1] = currentColor;
                    setStoragePos(1, true);
                } else if (SharedData.storage[2] == ColorSensed.NO_COLOR && currentColor != ColorSensed.NO_COLOR) {
                    SharedData.storage[2] = currentColor;
                    setStoragePos(2, true);
                }
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


            //april tag testing (I think)
            Pose2D aprilTagPose = new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0);
            Pose pedroPose = new Pose(0,0,0);
            if (robotPose() != null) {
                aprilTagPose = robotPose();
                Pose ftcStandard = PoseConverter.pose2DToPose(aprilTagPose, InvertedFTCCoordinates.INSTANCE);
                pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            }

            //telemetry for April Tag

            telemetry.addData("ROBOT APRIL X", aprilTagPose.getX(DistanceUnit.INCH));
            telemetry.addData("ROBOT APRIL Y", aprilTagPose.getY(DistanceUnit.INCH));
            telemetry.addData("ROBOT APRIL H", aprilTagPose.getHeading(AngleUnit.DEGREES));

            telemetry.addData("PEDRO X", pedroPose.getX());
            telemetry.addData("PEDRO Y", pedroPose.getY());
            telemetry.addData("PEDRO H", pedroPose.getPose().getHeading());

            telemetry.addData("F X", f.getPose().getX());
            telemetry.addData("F Y", f.getPose().getY());
            telemetry.addData("F H", f.getPose().getHeading());

            //telemetry for Color Sensor and Storage

//            telemetry.addData("Entrance Color", detectColor());
//            telemetry.addData("hue", JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
//            telemetry.addData("r", entranceColor.red());
//            telemetry.addData("g", entranceColor.green());
//            telemetry.addData("b", entranceColor.blue());
//            telemetry.addData("saturation", JavaUtil.rgbToSaturation(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
//            telemetry.addData("Storage", SharedData.storage[0] + ", " + SharedData.storage[1] + ", " + SharedData.storage[2]);
//            telemetry.addData("sort ticks", sortMotor.getCurrentPosition());


            SharedData.toTeleopPose = f.getPose();

            telemetry.update();
        }
    }

    public void setStoragePos(int slot, boolean intake){
        int ticks = 1426;
        if(intake) {
            if (slot == 0) {
                sortMotor.setTargetPosition(0);
            } else if (slot == 1) {
                sortMotor.setTargetPosition(ticks / 3);
            } else if (slot == 2) {
                sortMotor.setTargetPosition(2 * ticks / 3);
            }
        }else {
            if (slot == 0) {
                sortMotor.setTargetPosition(ticks/2);
            } else if (slot == 1) {
                sortMotor.setTargetPosition(-ticks/6);
            } else if (slot == 2) {
                sortMotor.setTargetPosition(ticks / 6);
            }
        }
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

    public ColorSensed detectColor() {
        double hue = JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        double saturation = JavaUtil.rgbToSaturation(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        if (hue < 180 && hue > 120 && saturation > .4)
            return ColorSensed.GREEN;
        if (hue > 200 && hue < 260 && saturation > .35)
            return ColorSensed.PURPLE;
        return ColorSensed.NO_COLOR;
    }

    public Pose2D robotPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                if (detection.metadata != null) {
                    rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    return new Pose2D(DistanceUnit.INCH, detection.rawPose.x, detection.rawPose.y, AngleUnit.DEGREES, rot.firstAngle);
                }
                else
                    telemetry.addData("Metadata", "null");
            }
        }
        return null;
    }

}


