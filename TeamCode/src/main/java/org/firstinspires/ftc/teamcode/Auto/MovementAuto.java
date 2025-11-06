package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.SharedData;

import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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
    private DcMotorEx fan = null;
    private DcMotorEx rightLaunch = null;
    private DcMotorEx leftLaunch = null;
    private DistanceSensor distanceSensor;

    boolean launching = false;
    boolean launch = true;

    int timesLaunched = 0;
    private ColorSensed previousColor = ColorSensed.NO_COLOR;
    private ColorSensed currentColor = ColorSensed.NO_COLOR;

    int slotGoal = 0;


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

    public Pose robotPose(){
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
        fan = hardwareMap.get(DcMotorEx.class, "sortMotor");
        feeder = hardwareMap.get(CRServo.class,"feederServo");
        rightLaunch = hardwareMap.get(DcMotorEx.class, "rightLaunch");
        leftLaunch = hardwareMap.get(DcMotorEx.class, "leftLaunch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        fan.setTargetPosition(0);
        fan.setTargetPositionTolerance(10);

        fan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fan.setPower(1);

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
        telemetry.addData("Sort Encoder", fan.getCurrentPosition());
        telemetry.addData("Storage", SharedData.storage[0] + ", " + SharedData.storage[1] + ", " + SharedData.storage[2]);
        telemetry.addData("launching", launching);
        telemetry.addData("cc", currentColor);
        telemetry.addData("pc", previousColor);
        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");
        panels.getTelemetry().update();


        telemetry.addData("Path State", pathState);

        //sets servo to intake
        if((pathState == 3|| pathState == 4||pathState == 6||pathState == 7) && !launching)
            intakeServo.setPower(1);

        else
            intakeServo.setPower(0);





        //detects color and sets storage position
        // may want to make faster

        currentColor = detectColor();
        //detectColorTimer.resetTimer();
        if(distanceSensor.getDistance(DistanceUnit.CM) < 2.7) {
            colorTimer.resetTimer();
        }


        //sets the slot position and color of the ball in storage
        if (!launching && previousColor != currentColor && colorTimer.getElapsedTimeSeconds() > .5) {
            colorTimer.resetTimer();

            if(currentColor != ColorSensed.NO_COLOR) {
                intakeTimer.resetTimer();
                if (SharedData.storage[0] == ColorSensed.NO_COLOR) {
                    SharedData.storage[0] = currentColor;
                    setStoragePos(0, true);
                } else if (SharedData.storage[1] == ColorSensed.NO_COLOR) {
                    SharedData.storage[1] = currentColor;
                    setStoragePos(1, true);
                } else if (SharedData.storage[2] == ColorSensed.NO_COLOR) {
                    SharedData.storage[2] = currentColor;
                    setStoragePos(2, true);
                }
            }
        }

        //swaps to open slot (if available)
        else if(!launching && colorTimer.getElapsedTimeSeconds() > .5){

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
        if(launching) {
            int ind = getPurpleIndex() != -1 ?  getPurpleIndex() : getInconclusiveIndex();
            if(timesLaunched == SharedData.greenIndex || ind == -1) {
                ind = getGreenIndex() == -1 ? ind : getGreenIndex();
            }


            if(ind != -1) {
                leftLaunch.setVelocity(2050);
                rightLaunch.setVelocity(2050);
                setStoragePos(ind, false);

            }
            if(leftLaunch.getVelocity() >= 1950 && rightLaunch.getVelocity() >= 1950 && launch) {
                feeder.setPower(1);
                launchTimer.resetTimer();
                launch = false;
            }
            else if(leftLaunch.getVelocity() <= 1950 && rightLaunch.getVelocity() <= 1950) {
                feeder.setPower(0);
            }
            if(launchTimer.getElapsedTimeSeconds() > 1.5 && !launch) {
                SharedData.storage[ind] = ColorSensed.NO_COLOR;
                timesLaunched ++;
                if(timesLaunched == 3)
                    timesLaunched = 0;
                launch = true;
            }
        }

        if(!launching) {
            feeder.setPower(0);
            leftLaunch.setVelocity(0);
            rightLaunch.setVelocity(0);
        }



    }

    @Override
    public void stop() {
        sendPose();
    }

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
        if(saturation > .55 || (green >= 110 && blue >= 100) || red > 100){
            if(red > 100)
                return ColorSensed.PURPLE;
            else if (green > blue)
                return ColorSensed.GREEN;
            else
                return ColorSensed.INCONCLUSIVE;
        }
        return ColorSensed.NO_COLOR;
    }

    public void setPathState(int pState) {
        this.pathState = pState;
        pathTimer.resetTimer();
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


    public int getInconclusiveIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.INCONCLUSIVE)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.INCONCLUSIVE)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.INCONCLUSIVE)
            temp = 2;
        return temp;
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
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > 1) {
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
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > 1) {
                    f.followPath(four, true);
                    setPathState(5);
                    sendPose();
                    launching = false;
                }else if(!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 )
                    launching = true;
                break;
            case 5:
                //Pickup 2
                if (!f.isBusy()) {
                    f.followPath(five, true);
                    f.setMaxPower(.2);
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
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 7) {
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