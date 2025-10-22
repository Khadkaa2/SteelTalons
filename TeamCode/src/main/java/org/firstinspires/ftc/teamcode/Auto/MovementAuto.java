package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;
import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Autonomous")
public class MovementAuto extends OpMode {

    private Follower f;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    private Timer pathTimer, actionTimer, opmodeTimer, colorTimer, launchTimer,intakeTimer, detectColorTimer;
    private int pathState;
    private PoseConstants poses = new PoseConstants();
    Pose currentPose = null;
    private Path start, end, p, point;
    private PathChain one, two, three, four, five, six;
    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    int index;
    private CRServo intakeServo = null;
    private CRServo feeder = null;
    private ColorSensor entranceColor;
    private DcMotorEx sortMotor = null;
    boolean launching = false;
    boolean intaking = false;
    int timesLaunched = 0;
    private ColorSensed previousColor = ColorSensed.NO_COLOR;
    private ColorSensed currentColor = ColorSensed.NO_COLOR;



    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public static int figureID(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 21||detection.id == 22||detection.id == 23)
                return detection.id;
        }
        return -1;
    }

    public  Pose robotPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 20||detection.id == 24){
                if(detection.metadata!= null)
                    return new Pose( detection.robotPose.getPosition().x , detection.robotPose.getPosition().y , detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
//                            Pose((0) + Math.sin(f.getPose().getX() - detection.ftcPose.x) - poses.CamOff.getX(), (0) + Math.cos(detection.ftcPose.y) - poses.CamOff.getY(),(detection.ftcPose.yaw));
                else
                    telemetry.addData("Metadata", "null");
            }
        }

        return null;
    }

    public ColorSensed detectColor() {
        double hue = JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        double saturation = JavaUtil.rgbToSaturation(entranceColor.red(), entranceColor.green(), entranceColor.blue());
        if (hue < 180 && hue > 120 && saturation > .6)
            return ColorSensed.GREEN;
        if (hue > 200 && hue < 260 && saturation > .55)
            return ColorSensed.PURPLE;
        return ColorSensed.NO_COLOR;
    }

    public void setPathState(int pState) {
        this.pathState = pState;
        pathTimer.resetTimer();
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
        }else{
            if (slot == 0) {
                sortMotor.setTargetPosition(ticks/2);
            } else if (slot == 1) {
                sortMotor.setTargetPosition(-ticks/6);
            } else if (slot == 2) {
                sortMotor.setTargetPosition(ticks / 6);
            }
        }
    }

    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();


        panels.getTelemetry().addData("Path State", pathState);
        panels.getTelemetry().addData("Turning", f.isTurning());
        panels.getTelemetry().addData("headingError", f.getHeadingError());
        panels.getTelemetry().addData("heading", f.getPose().getHeading());
        panels.getTelemetry().addData("x", f.getPose().getX());
        panels.getTelemetry().addData("y", f.getPose().getY());
        telemetry.addData("Sort Encoder", sortMotor.getCurrentPosition());
        telemetry.addData("Storage", SharedData.storage[0] + ", " + SharedData.storage[1] + ", " + SharedData.storage[2]);
        telemetry.addData("launching", launching);
        telemetry.addData("cc", currentColor);
        telemetry.addData("pc", previousColor);
        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");
        panels.getTelemetry().update();


        telemetry.addData("Path State", pathState);

        //sets servo to intake
        if(pathState == 3||pathState == 6)
            intakeServo.setPower(1);
        else
            intakeServo.setPower(0);





        //detects color and sets storage position
        if(detectColorTimer.getElapsedTimeSeconds()>.2) {
            currentColor = detectColor();
            detectColorTimer.resetTimer();
        }

        //sets the slot position and color of the ball in storage
        if (!launching && previousColor != currentColor && colorTimer.getElapsedTimeSeconds() > .5) {
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
        else if(!launching && currentColor == ColorSensed.NO_COLOR && colorTimer.getElapsedTimeSeconds() > .5){

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

        //detects if ready to launch and set storage position
        //need to adapt code so that if multiple greens/ not enough purples are inputted, it will still launch what it has
        if(launching && launchTimer.getElapsedTimeSeconds()>1) {
            int ind = -1;
            if(timesLaunched == SharedData.greenIndex) {
                ind = getGreenIndex();
            }
            else
            {
                ind = getPurpleIndex();
            }

            if(ind != -1) {
                feeder.setPower(-1);
                setStoragePos(ind, false);
                timesLaunched ++;
                if(timesLaunched == 3)
                    timesLaunched = 0;
                launchTimer.resetTimer();
                SharedData.storage[ind] = ColorSensed.NO_COLOR;
            }
        }

        if(!launching)
            feeder.setPower(0);

        

    }


    //gets the index of a green ball (not the closest)
    //Need to adapt code to set position to nearest green ball
    public int getGreenIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.GREEN)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.GREEN)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.GREEN)
            temp = 2;
        return temp;
    }

    //gets the index of a purple ball (not the closest)
    //Need to adapt code to set position to nearest purple ball
    public int getPurpleIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.PURPLE)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.PURPLE)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.PURPLE)
            temp = 2;
        return temp;
    }

    @Override
    public void init() {

        SharedData.reset();
        initAprilTag();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        intakeTimer = new Timer();
        detectColorTimer = new Timer();
        colorTimer = new Timer();
        launchTimer = new Timer();
        opmodeTimer.resetTimer();

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(poses.START_POSE);
        f.activateAllPIDFs();

        buildPaths();

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        sortMotor = hardwareMap.get(DcMotorEx.class, "sortMotor");
        feeder = hardwareMap.get(CRServo.class,"feederServo");

        sortMotor.setTargetPosition(0);
        sortMotor.setTargetPositionTolerance(10);

        sortMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sortMotor.setPower(1);

        intakeServo.setPower(0);
        index = 0;
    }

    @Override
    public void init_loop() {
        //detect AprilTag motif and set green index in SharedData
        int ID = figureID();

        if (ID == 21) {
            index = 0;
        } else if (ID == 22) {
            index = 1;
        } else if (ID == 23)
            index = 2;

        SharedData.greenIndex = index;
        telemetry.addData("Green Index", index );
        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");


        currentPose = robotPose();
        if (currentPose!= null){
            telemetry.addData("X",currentPose.getX());
            telemetry.addData("Y",currentPose.getY());
            telemetry.addData("H",currentPose.getHeading());
        }

        telemetry.update();
    }

    @Override
    public void start() {
        sendPose();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        sendPose();
    }

    public void buildPaths() {
        start = new Path(new BezierLine(poses.START_POSE, poses.LAUNCH_POSE));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading(), poses.LAUNCH_POSE.getHeading());



        one = f.pathBuilder()
                .addPath(new BezierLine(poses.LAUNCH_POSE, poses.ALIGN1_POSE))
                .setConstantHeadingInterpolation(poses.ALIGN1_POSE.getHeading())
                .build();
        two = f.pathBuilder()
                .addPath(new BezierLine(poses.ALIGN1_POSE, poses.PICKUP1_POSE))
                .setLinearHeadingInterpolation(poses.ALIGN1_POSE.getHeading(), poses.PICKUP1_POSE.getHeading())
                .build();
        three = f.pathBuilder()
                .addPath(new BezierLine(poses.PICKUP1_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.PICKUP1_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();
        four = f.pathBuilder()
                .addPath(new BezierLine(poses.LAUNCH_POSE, poses.ALIGN2_POSE))
                .setConstantHeadingInterpolation(poses.ALIGN2_POSE.getHeading())
                .build();
        five = f.pathBuilder()
                .addPath(new BezierLine(poses.ALIGN2_POSE, poses.PICKUP2_POSE))
                .setLinearHeadingInterpolation(poses.ALIGN2_POSE.getHeading(), poses.PICKUP2_POSE.getHeading())
                .build();
        six = f.pathBuilder()
                .addPath(new BezierLine(poses.PICKUP2_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.PICKUP2_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();
        end = new Path(new BezierLine(poses.LAUNCH_POSE, poses.END_POSE));
        end.setLinearHeadingInterpolation(poses.LAUNCH_POSE.getHeading(), poses.END_POSE.getHeading());
    }

    private void sendPose(){
        SharedData.toTeleopPose = f.getPose();

    }

    public void autoPathUpdates() {
        sendPose();
        switch (pathState) {
            case 0:
                //Score 1
                f.followPath(start);
                setPathState(1);
                sendPose();
                break;
            case 1:
                //Align 1
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    f.followPath(one, true);
                    setPathState(2);
                    sendPose();
                    launching = false;
                } else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)
                    launching = true;
                break;
            case 2:
                //Pickup 1
                if (!f.isBusy()) {
                    f.followPath(two, true);
                    f.setMaxPower(.2);
                    setPathState(3);
                    sendPose();
                }
                break;
            case 3:
                //Score 2
                if (!f.isBusy()) {
                    f.followPath(three, true);
                    f.setMaxPower(1);
                    setPathState(4);
                    sendPose();
                }
                break;
            case 4:
                //Align 2
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    f.followPath(four, true);
                    setPathState(5);
                    sendPose();
                    launching = false;
                }else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)
                    launching = true;
                break;
            case 5:
                //Pickup 2
                if (!f.isBusy()) {
                    f.followPath(five, true);
                    f.setMaxPower(.15);
                    setPathState(6);
                    sendPose();
                }
                break;
            case 6:
                //Score 3
                if (!f.isBusy()) {
                    f.followPath(six, true);
                    f.setMaxPower(1);
                    setPathState(7);
                    sendPose();
                }
                break;
            case 7:
                //move to human area
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    f.followPath(end, true);
                    setPathState(8);
                    sendPose();
                    launching = false;
                }else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)
                    launching = true;
                break;
            case 8:
                if (!f.isBusy()) {
                    setPathState(-1);
                    sendPose();
                }
                break;
        }
    }
}