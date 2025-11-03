package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Color;

import androidx.xr.runtime.math.Pose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class NewAuto extends OpMode {

    private Robot hornet = new Robot(hardwareMap);
    private Follower f;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    private int pathState;
    private Timer pathTimer, actionTimer,opmodeTimer,colorTimer,launchTimer,intakeTimer,detectColorTimer
    private PoseConstants poses = new PoseConstants();
    Pose currentPose = null;
    private Path start, end;
    private PathChain one, two, three, four, five, six;
    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    int index;

    boolean launching = false , launch = true;

    int timesLaunched = 0;
    private ColorSensed previousColor = ColorSensed.NO_COLOR, currentColor = ColorSensed.NO_COLOR;

    int slotGoal = 0;

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
        buildPaths();


        index = 0;

    }

    @Override
    public void init_loop() {
    int ID = figureID();
    if (ID == 21) index = 0;
    else if (ID == 22) index = 1;
    else if (ID == 23) index = 2;
    SharedData.greenIndex = index;

        telemetry.addData("Green Index", index );
        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");

    }

    @Override
    public void start(){
        sendPose();
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();

        // panels.getTelemetry().addData("key", value);
        // panels.getTelemetry().update();
        if ((pathState == 3 || pathState == 4 || pathState == 6 || pathState == 7) && !launching) {
            hornet.startIntake(true);
        } else hornet.stopIntake();

        currentColor = detectColor();

        //SET LAUNCHING POSITION
        if (!launching && previousColor != currentColor && colorTimer.getElapsedTimeSeconds() > .5) {
            colorTimer.resetTimer();

            if(currentColor != ColorSensed.NO_COLOR) {
                intakeTimer.resetTimer();
                if (SharedData.storage[0] == ColorSensed.NO_COLOR) {
                    SharedData.storage[0] = currentColor;
                    hornet.setStoragePos(0, true);
                } else if (SharedData.storage[1] == ColorSensed.NO_COLOR) {
                    SharedData.storage[1] = currentColor;
                    hornet.setStoragePos(1, true);
                } else if (SharedData.storage[2] == ColorSensed.NO_COLOR) {
                    SharedData.storage[2] = currentColor;
                    hornet.setStoragePos(2, true);
                }
            }
        }

        //swaps to open slot (if available)
        else if(!launching && colorTimer.getElapsedTimeSeconds() > .5){

            if(SharedData.storage[0] == ColorSensed.NO_COLOR)
                hornet.setStoragePos(0, true);
            else if(SharedData.storage[1] == ColorSensed.NO_COLOR)
                hornet.setStoragePos(1, true);
            else if(SharedData.storage[2] == ColorSensed.NO_COLOR)
                hornet.setStoragePos(2, true);
            else
                hornet.setStoragePos(1,false);
        }
        previousColor = currentColor;
        if(launching) {
            int ind = SharedData.getPurpleIndex() != -1 ?  getPurpleIndex() : getInconclusiveIndex();
            if(timesLaunched == SharedData.greenIndex || ind == -1) {
                ind = SharedData.getGreenIndex() == -1 ? ind : SharedData.getGreenIndex();
            }


            if(ind != -1) {
                hornet.startLaunchMotors(true);
                hornet.setStoragePos(ind, false);

            }
            if(hornet.atTargetVelocity() && launch) {
                hornet.startFeeder(true);
                launchTimer.resetTimer();
                launch = false;
            }
            else if(!hornet.atTargetVelocity()) {
                hornet.stopFeeder();
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
            hornet.stopFeeder();
            hornet.stopLaunchMotors();
        }
    }








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




}
