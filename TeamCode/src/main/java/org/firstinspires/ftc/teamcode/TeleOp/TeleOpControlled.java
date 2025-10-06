package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;

import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;

@TeleOp
public class TeleOpControlled extends LinearOpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private CRServo intakeServo;
    private ColorSensor entranceColor;



    private PoseConstants poses =  new PoseConstants();
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

    public enum ColorSensed
    {
        GREEN,
        PURPLE,
        NO_COLOR
    }

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");

        ColorSensed[] storage = {ColorSensed.NO_COLOR,ColorSensed.NO_COLOR,ColorSensed.NO_COLOR};

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose ==null ? new Pose() : SharedData.toTeleopPose);
        f.update();

        toLaunch = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(),poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(f.getPose().getHeading() , poses.LAUNCH_POSE.getHeading())
                .build();

        toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.parkPose))
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
            if (!automated ) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        false
                );
            }

            //Slow mode
            if(gamepad1.y && slowDelay.getElapsedTimeSeconds()>.5) {
                slowDelay.resetTimer();
                if(speedMultiplier == 1)
                    speedMultiplier = .1;
                else
                    speedMultiplier = 1;
            }

            //Auto Pathing to Launch
            if (gamepad1.a && !automated){
                f.followPath(toLaunch);
                automated = true;

            }
            //Auto Pathing to Park
            else if (gamepad1.x && !automated){
                f.followPath(toPark);
                automated = true;

            }
            //exits automated pathing
            else if ((!gamepad1.a && !gamepad1.x) && automated) {
                f.startTeleopDrive(true);
                brakeMotors();
                automated=false;
            }




            setCurr();

            //Intake
            if(gamepad1.right_bumper)
                intakeServo.setPower(1);
            else if (gamepad1.left_bumper)
                intakeServo.setPower(-1);
            else
                intakeServo.setPower(0);


            ColorSensed currentColor = detectColor();
            if(previousColor != currentColor) {
                if(storage[0] == ColorSensed.NO_COLOR)
                    storage[0] = currentColor;
                else if(storage[1] == ColorSensed.NO_COLOR)
                    storage[1] = currentColor;
                else if(storage[2] == ColorSensed.NO_COLOR)
                    storage[2] = currentColor;
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
            telemetry.addData("Entrance Color", detectColor());
            telemetry.addData("hue", JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
            telemetry.addData("Storage", storage[0] + ", " + storage[1] + ", "+storage[2]);
            SharedData.toTeleopPose = f.getPose();

            telemetry.update();
        }
    }

    private void brakeMotors(){
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void setCurr(){
        ArrayList<AprilTagDetection> detections = new ArrayList<>();

        try {
            this.currentDetection = detections.get(0);
        }
        catch (Exception e) {
            telemetry.addData("NO TAG DETECTED", true);
        }
    }

    public ColorSensed detectColor()
    {
        double hue = JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        if(hue < 180 && hue > 120)
            return ColorSensed.GREEN;
        if(hue > 200 && hue < 260)
            return ColorSensed.PURPLE;
        return ColorSensed.NO_COLOR;
    }

}




