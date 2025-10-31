package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


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
    private DcMotorEx fan = null;
    private CRServo intakeServo;
    private CRServo feeder;
    private DcMotorEx rightLaunch = null;
    private DcMotorEx leftLaunch = null;

    private ColorSensor entranceColor;
    private Servo launchAngle;


    private PoseConstants poses = new PoseConstants();
    private Follower f;
    private boolean automated = false;
    private boolean intaking = false;
    private PathChain toLaunchSame, toPark, toLaunchCross;

    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private Orientation rot;

    //private static AprilTagProcessor aprilTag;
    //private VisionPortal visionPortal;
    //private AprilTagDetection currentDetection;

    private double speedMultiplier;

    private Timer slowDelay, colorTimer, launchTimer, detectColorTimer, intakeTimer;

    private ColorSensed previousColor = ColorSensed.NO_COLOR;

    ColorSensed currentColor = ColorSensed.NO_COLOR;

    private boolean manual = false;

    private boolean feederFirstTime = true;

    int slotGoal = 0;

    public void runOpMode() throws InterruptedException {


        //init hardware
        fan = hardwareMap.get(DcMotorEx.class, "sortMotor");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        feeder = hardwareMap.get(CRServo.class, "feederServo");
        rightLaunch = hardwareMap.get(DcMotorEx.class, "rightLaunch");
        leftLaunch = hardwareMap.get(DcMotorEx.class, "leftLaunch");
        //servo = hardwareMap.get(Servo.class, "angleMotor");

        //make launchmotors track pos and brake
        rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);



        //init fan
        fan.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fan.setTargetPosition(fan.getCurrentPosition());
        fan.setTargetPositionTolerance(5);
        fan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fan.setPower(1);

        //init follower
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose == null ? poses.START_POSE : SharedData.toTeleopPose);
        f.update();

        //create paths
        toLaunchSame = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        toLaunchCross = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.teleOpLaunchPose))
                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.teleOpLaunchPose.getHeading())
                .build();

        toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.parkPose))
                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
                .build();

//        initAprilTag();

        //init timers/constants
        speedMultiplier = 1;
        slowDelay = new Timer();
        colorTimer = new Timer();
        launchTimer = new Timer();
        detectColorTimer = new Timer();
        intakeTimer = new Timer();
        colorTimer.resetTimer();
        slowDelay.resetTimer();
        launchTimer.resetTimer();
        detectColorTimer.resetTimer();
        intakeTimer.resetTimer();

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
                        false,
                        SharedData.red ? 0 : Math.toRadians(180)
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
                f.followPath(toLaunchSame);
                automated = true;
            }
            //Auto Pathing to Park
            else if (gamepad1.x && !automated) {
                f.followPath(toPark);
                automated = true;
            }
            else if (gamepad1.b && !automated) {
                f.followPath(toLaunchCross);
                automated = true;
            }
            //exits automated pathing
            else if ((!gamepad1.a && !gamepad1.x) && automated) {
                f.startTeleopDrive(true);
                automated = false;
            }


            //Intake powers
            //may want to set it so that it is always slightly intakes if necessary later
            if (gamepad1.left_bumper || (intakeTimer.getElapsedTimeSeconds() < .75 && !intaking))
                intakeServo.setPower(-1);
            else if (gamepad1.right_bumper || (intakeTimer.getElapsedTimeSeconds() < .75 && intaking))
                intakeServo.setPower(1);
            else
                intakeServo.setPower(0);

            if (manual)
                manualMode();
            else
                autoMode();

            if (gamepad2.xWasPressed())
                manual = !manual;


            tele();
            SharedData.toTeleopPose = f.getPose();

        }
    }
    public void setStoragePos(int slot, boolean intake) {
        int ticks = 1426;
        int absolutePos = fan.getCurrentPosition();
        int relativePos = absolutePos % ticks;
        int rotationOffset = absolutePos-relativePos;

        int sign = (int)Math.signum(absolutePos == 0 ? 1 : absolutePos);
        slotGoal = slot;
        if (intake) {
            if (slot == 0) {
               if(relativePos <= ticks/2)
                   fan.setTargetPosition(rotationOffset);
               else
                   fan.setTargetPosition(rotationOffset + ticks);
            } else if (slot == 1) {
                if(relativePos <= 5*ticks/6)
                    fan.setTargetPosition(rotationOffset + ticks/3);
                else
                    fan.setTargetPosition(rotationOffset + 4*ticks/3);
            } else if (slot == 2) {
               if(relativePos >= ticks/6)
                   fan.setTargetPosition(rotationOffset + 2*ticks/3);
               else
                   fan.setTargetPosition(rotationOffset - ticks/3);

            }
        }
        else {
            if (slot == 0) {
                fan.setTargetPosition(rotationOffset + ticks/2);
            } else if (slot == 1) {
                if(relativePos >= ticks/3)
                    fan.setTargetPosition(rotationOffset + 5*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset - ticks/6);
            } else if (slot == 2) {
                if(relativePos >= 2*ticks/3)
                    fan.setTargetPosition(rotationOffset + 7*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset + ticks/6);
            }
        }
    }

//    public void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//    }

    public ColorSensed detectColor() {
        int red = entranceColor.red();
        int blue = entranceColor.blue();
        int green = entranceColor.green();
        double hue = JavaUtil.rgbToHue(red, green, blue);
        double saturation = JavaUtil.rgbToSaturation(red, green, blue);
        if (hue < 180 && hue > 120 && saturation > .6)
            return ColorSensed.GREEN;
        if (hue > 200 && hue < 260 && saturation > .55)
            return ColorSensed.PURPLE;
        if(saturation > .55 || green >= 110 || blue >= 100)

            if(green > blue)
                return ColorSensed.GREEN;
            else
                return ColorSensed.PURPLE;
        return ColorSensed.NO_COLOR;
    }

    public void tele() {
//            telemetry.addData("FOLLOWER X",f.getPose().getX());
//            telemetry.addData("FOLLOWER Y",f.getPose().getY());
////            telemetry.addData("FOLLOWER Heading",f.getPose().getHeading());
////            telemetry.addData("TAGX", currentDetection.ftcPose.x );
////            telemetry.addData("TAGY", currentDetection.ftcPose.y );
////            telemetry.addData("TAGH", currentDetection.ftcPose.yaw);
////            telemetry.addData("TAGB", currentDetection.ftcPose.bearing);
////            telemetry.addData("TAGR" , currentDetection.ftcPose.range);
//
//
////            telemetry for April Tag
////
////            telemetry.addData("ROBOT APRIL X", aprilTagPose.getX(DistanceUnit.INCH));
////            telemetry.addData("ROBOT APRIL Y", aprilTagPose.getY(DistanceUnit.INCH));
////            telemetry.addData("ROBOT APRIL H", aprilTagPose.getHeading(AngleUnit.DEGREES));
////
////            telemetry.addData("PEDRO X", pedroPose.getX());
////            telemetry.addData("PEDRO Y", pedroPose.getY());
////            telemetry.addData("PEDRO H", pedroPose.getPose().getHeading());
////
////            telemetry.addData("F X", f.getPose().getX());
////            telemetry.addData("F Y", f.getPose().getY());
////            telemetry.addData("F H", f.getPose().getHeading());
//
//        //telemetry for Color Sensor and Storage
//
//            telemetry.addData("Entrance Color", detectColor());
//            telemetry.addData("hue", JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
//            telemetry.addData("r", entranceColor.red());
//            telemetry.addData("g", entranceColor.green());
//            telemetry.addData("b", entranceColor.blue());
//            telemetry.addData("saturation", JavaUtil.rgbToSaturation(entranceColor.red(), entranceColor.green(), entranceColor.blue()));
//            telemetry.addData("sort ticks", fan.getCurrentPosition());
//            telemetry.addData("Color Timer", colorTimer.getElapsedTimeSeconds());
//            telemetry.addData("Launch Timer", launchTimer.getElapsedTimeSeconds());
        //telemetry.addData("Pattern", SharedData.greenIndex);
        telemetry.addLine(String.format("Storage: %s, %s, %s", SharedData.storage[0], SharedData.storage[1], SharedData.storage[2] ));
        telemetry.addData("Manual Mode", manual);
//        telemetry.addData("fanPos", fan.getCurrentPosition());
//        telemetry.addData("fanTarget", fan.getTargetPosition());
//        telemetry.addData("Slot Goal", slotGoal);
        telemetry.addData("Side", !SharedData.red ? "Red" : "Blue");
//        telemetry.addLine(String.format("LeftVel: %f\nRightVel: %f",leftLaunch.getVelocity(), rightLaunch.getVelocity() ));

        telemetry.update();
    }

//    public Pose2D robotPose() {
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        for (AprilTagDetection detection : detections) {
//            if (detection.id == 20 || detection.id == 24) {
//                if (detection.metadata != null) {
//                    rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//                    return new Pose2D(DistanceUnit.INCH, detection.rawPose.x, detection.rawPose.y, AngleUnit.DEGREES, rot.firstAngle);
//                }
//                else
//                    telemetry.addData("Metadata", "null");
//            }
//        }
//        return null;
//    }

    public void manualMode() {

        //this is the code for all the manualmode in teleop. There will be no automation for player2 in this mode

        //fan control
        if (gamepad2.a) {
            setStoragePos(0, !gamepad2.dpad_left);
        } else if (gamepad2.b) {
            setStoragePos(1, !gamepad2.dpad_left);
        } else if (gamepad2.y) {
            setStoragePos(2, !gamepad2.dpad_left);
        }

      //feeder
        if (gamepad2.left_trigger >= .2) {
            feeder.setPower(1);
        }
        else{
            feeder.setPower(0);
        }
        
        
        // Two ways of manually moving the fan

        //sticks 
        if (gamepad2.dpad_down) {
            if (gamepad2.start){
                fan.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                fan.setTargetPosition(fan.getCurrentPosition());
                fan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("START" , gamepad2.start);
            }
            else fan.setTargetPosition((int) (fan.getCurrentPosition() + gamepad2.right_stick_x * 10));
        }

        leftLaunch.setVelocity(gamepad2.right_trigger * 2500);
        rightLaunch.setVelocity(gamepad2.right_trigger * 2500);




    }

    public void autoMode () {

        //manual color setting
        //hold right bumper and press a button for green
        //hold left bumper and press a button for purple
        //"a" for slot 0
        //"b" for slot 1
        //"y" for slot 2
        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.NO_COLOR;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.NO_COLOR;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.NO_COLOR;
        }
        else if(gamepad2.right_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.GREEN;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.GREEN;
            else if(gamepad2.y)
            SharedData.storage[2] = ColorSensed.GREEN;
        }
        else if(gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.PURPLE;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.PURPLE;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.PURPLE;
        }




        //Checks if it has been enough time since last launch
        //dpad up for green
        //dpad down for purple
        //dpad left to clear storage (for testing)
        if(launchTimer.getElapsedTimeSeconds()>2){
            //launches green
            if (gamepad2.dpad_up) {

                int ind = -1;
                if (SharedData.storage[0] == ColorSensed.GREEN)
                    ind = 0;
                else if (SharedData.storage[1] == ColorSensed.GREEN)
                    ind = 1;
                else if (SharedData.storage[2] == ColorSensed.GREEN)
                    ind = 2;
                if (ind != -1) {
                    launchTimer.resetTimer();
                    setStoragePos(ind, false);
                    SharedData.storage[ind] = ColorSensed.NO_COLOR;
                }
            }

            //launches purple
            else if (gamepad2.dpad_down) {
                int ind = -1;
                if (SharedData.storage[0] == ColorSensed.PURPLE)
                    ind = 0;
                else if (SharedData.storage[1] == ColorSensed.PURPLE)
                    ind = 1;
                else if (SharedData.storage[2] == ColorSensed.PURPLE)
                    ind = 2;
                if (ind != -1) {
                    launchTimer.resetTimer();
                    setStoragePos(ind, false);
                    SharedData.storage[ind] = ColorSensed.NO_COLOR;
                }
            }
            //empties storage
            else if (gamepad2.dpad_left){
                SharedData.emptyStorage();
            }
        }
        if(launchTimer.getElapsedTimeSeconds()<2) {
            if(!feederFirstTime) {
                if(rightLaunch.getVelocity() >= 2200 && leftLaunch.getVelocity() >= 2200)
                    feeder.setPower(1);
                rightLaunch.setVelocity(2250);
                leftLaunch.setVelocity(2250);
            }
        }
        else {
            feeder.setPower(0);
            rightLaunch.setVelocity(0);
            leftLaunch.setVelocity(0);
            feederFirstTime = false;
        }
        //Senses if a ball is in the intake area
        //Sets sorting position to open area on detection
        //only detects every .2 seconds to reduce input lag
        if(detectColorTimer.getElapsedTimeSeconds()>.1) {
            currentColor = detectColor();
            detectColorTimer.resetTimer();
        }

        //sets the slot position and color of the ball in storage
        if (previousColor != currentColor && colorTimer.getElapsedTimeSeconds() > .5) {
            colorTimer.resetTimer();

            if(currentColor != ColorSensed.NO_COLOR) {
                intakeTimer.resetTimer();
                intaking = true;
                if (SharedData.storage[0] == ColorSensed.NO_COLOR) {
                    SharedData.storage[0] = currentColor;
                    setStoragePos(0, true);
                } else if (SharedData.storage[1] == ColorSensed.NO_COLOR) {
                    SharedData.storage[1] = currentColor;
                    setStoragePos(1, true);
                } else if (SharedData.storage[2] == ColorSensed.NO_COLOR) {
                    SharedData.storage[2] = currentColor;
                    setStoragePos(2, true);
                }else {
                    intaking = false;
                }
            }
        }

        //swaps to open slot (if available)
        else if(SharedData.storage[slotGoal] != ColorSensed.NO_COLOR && colorTimer.getElapsedTimeSeconds() > .5 && launchTimer.getElapsedTimeSeconds() > 2){

            if(SharedData.storage[0] == ColorSensed.NO_COLOR)

                setStoragePos(0, true);
            else if(SharedData.storage[1] == ColorSensed.NO_COLOR)
                setStoragePos(1, true);
            else if(SharedData.storage[2] == ColorSensed.NO_COLOR)
                setStoragePos(2, true);
            else
                setStoragePos(1,false);
        }
        previousColor = currentColor;

        //april tag testing (I think)
//            Pose2D aprilTagPose = new Pose2D(DistanceUnit.INCH, 0,0,AngleUnit.DEGREES,0);
//            Pose pedroPose = new Pose(0,0,0);
//            if (robotPose() != null) {
//                aprilTagPose = robotPose();
//                Pose ftcStandard = PoseConverter.pose2DToPose(aprilTagPose, InvertedFTCCoordinates.INSTANCE);
//                pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//            }
    }

}


