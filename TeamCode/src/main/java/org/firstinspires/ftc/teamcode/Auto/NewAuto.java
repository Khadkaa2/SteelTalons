package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Color;

import androidx.xr.runtime.math.Pose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous (name = "AAA Auto")
public class NewAuto extends OpMode {

    private Robot hornet = new Robot(hardwareMap);
    private Follower f;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    private int pathState;
    private Timer pathTimer,opmodeTimer,launchTimer,detectColorTimer;
    private PoseConstants poses = new PoseConstants();
    Pose currentPose = null;
    private Path start, end;
    private PathChain one, two, three, four, five, six;
    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    int index;

    boolean launching = false , launchingTemp = false;

    int timesLaunched = 0;

    int slotGoal = 0;

    @Override
    public void init() {
        SharedData.reset();
        initAprilTag();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
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
    telemetry.update();
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

        //SET STORAGE COLOR
        if(hornet.buttonPressed() && hornet.atSortTarget() && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            SharedData.storage[hornet.getSlotGoal()] = hornet.detectColor();
        }

        if(launching && !SharedData.isEmpty() && !launchingTemp) {
            int ind = SharedData.getPurpleIndex();
            if(timesLaunched == SharedData.greenIndex || ind == -1) {
                ind = SharedData.getGreenIndex() == -1 ? ind : SharedData.getGreenIndex();
            }
            hornet.setStoragePos(ind,false);
            launchingTemp = true;
        }
        if(launchingTemp && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.flapAtLaunch()){
            hornet.launch();
            launchTimer.resetTimer();
        }

        if(launchTimer.getElapsedTimeSeconds() >= .25){
            if(hornet.flapAtLaunch()){
                launchTimer.resetTimer();
                hornet.resetFlap();
                SharedData.clearSlot(hornet.getSlotGoal());
            }
            else if(launchingTemp && hornet.isLaunched()){
                launchingTemp = false;
                hornet.resetLaunch();
            }
        }

        if(!launching) {
            hornet.resetFlap();
            hornet.resetLaunch();
            hornet.stopLaunchMotors();
            hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , SharedData.isFull());
        }else{hornet.startLaunchMotors(true);}
    }

    public void stop(){
        sendPose();
    }

    public void setPathState(int state){
        this.pathState = state;
        pathTimer.resetTimer();
    }


    public void buildPaths(){
        start = new Path( new BezierLine(poses.START_POSE , poses.LAUNCH_POSE));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading() , poses.LAUNCH_POSE.getHeading());

        one = f.pathBuilder()
                .addPath(new BezierLine( poses.LAUNCH_POSE , poses.ALIGN1_POSE))
                .setConstantHeadingInterpolation(poses.ALIGN1_POSE.getHeading())
                .build();
        two = f.pathBuilder()
                .addPath(new BezierLine( poses.ALIGN1_POSE , poses.PICKUP1_POSE))
                .setLinearHeadingInterpolation( poses.ALIGN1_POSE.getHeading() , poses.PICKUP1_POSE.getHeading())
                .build();
        three = f.pathBuilder()
                .addPath(new BezierLine(poses.PICKUP1_POSE , poses.LAUNCH_POSE ))
                .setLinearHeadingInterpolation(poses.PICKUP1_POSE.getHeading() , poses.LAUNCH_POSE.getHeading())
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

    public void autoPathUpdates(){
        sendPose();
        switch (pathState){
            case 0:
                //go to scoring pos
                f.followPath(start);
                setPathState(1);
                sendPose();
                break;
            case 1:
                //go to align 1 pos
                sendPose();
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > 1 ){
                    f.followPath(one, true);
                    setPathState(2);
                    sendPose();
                    launching = false;
                }
                //score 1
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1)
                    launching = true;
                break;
            case 2:
                //pickup balls
                if (!f.isBusy()){
                    f.followPath(two, true);
                    f.setMaxPower(.2);
                    setPathState(3);
                    sendPose();
                }
                break;
            case 3:
                //go to scoring pos
                if (!f.isBusy()){
                    f.followPath(three, true);
                    f.setMaxPower(1);
                    setPathState(4);
                    sendPose();
                }
                break;
            case 4:
                //go to align 2
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > 1) {
                    f.followPath(four, true);
                    setPathState(5);
                    sendPose();
                    launching = false;
                }
                //score                         can we remove this part?   //
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    launching = true;
                }
                break;
            case 5:
                // pickup 2
                if (!f.isBusy()){
                    f.followPath(five , true);
                    f.setMaxPower(.2);
                    setPathState(6);
                    sendPose();
            }
                break;
            case 6:
                //move to score pos
                if (!f.isBusy()){
                    f.followPath(six , true);
                    f.setMaxPower(1);
                    sendPose();
                    setPathState(7);
                }
                break;
            case 7:
                // sends to final location
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > 1){
                    f.followPath(end);
                    setPathState(8);
                    sendPose();
                }
                //score
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() >1){
                    launching = true;
                }
                break;
            case 8:
                if (!f.isBusy()){
                    setPathState(-1);
                    sendPose();
                }
                break;
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

    public void sendPose(){
        SharedData.toTeleopPose = f.getPose();
    }


}
