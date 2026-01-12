package org.firstinspires.ftc.teamcode.Auto;

import androidx.xr.runtime.math.Pose;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "9 alt auto")
public class Auto9BallAlt extends OpMode {

    private Robot hornet = new Robot();
    private Follower f = null;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    private int pathState;
    private Timer pathTimer,opmodeTimer,launchTimer,detectColorTimer;
    private PoseConstants poses = new PoseConstants();
    Pose currentPose = null;
    private Path start, end;
    private PathChain one, two, three, four, five, six;
    int index;
    private Limelight3A limelight;
    boolean launching = false , launchingTemp = false;
    private LLResult result;
    int timesLaunched = 0;

    @Override
    public void init() {
        hornet.initialize(this.hardwareMap);
        SharedData.reset();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        launchTimer = new Timer();
        opmodeTimer.resetTimer();
        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(poses.START_POSE);
        buildPaths();
        hornet.resetHammer();
        hornet.resetLaunch();
        limelight.start();
        index = 0;

    }

    @Override
    public void init_loop() {
        int ID = figureID();
        if (ID == 21) index = 0;
        else if (ID == 22) index = 1;
        else if (ID == 23) index = 2;

        SharedData.greenIndex = index;
        telemetry.addData("ID" , ID);
        telemetry.addData("Green Index", index );
        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");
        telemetry.addData("Start", SharedData.startFar ? "far" : "close");
        telemetry.addData("Shoot", SharedData.shootFar ? "far" : "close");

    /*if (gamepad2.a){
        limelight.start();
    }
    else if (gamepad2.b){
        limelight.stop();
    }*/

        telemetry.update();
    }

    @Override
    public void start(){
        sendPose();
        opmodeTimer.resetTimer();
        limelight.stop();
        setPathState(0);
        hornet.updateLED();
    }
    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();
        hornet.updateLED();

        if(!launching) {
            hornet.resetHammer();
            hornet.resetLaunch();
            if(pathState == 1 || pathState == 4 || pathState == 7)
                hornet.startLaunchMotors(SharedData.shootFar);
            else
                hornet.stopLaunchMotors();
            if(launchTimer.getElapsedTimeSeconds() > .1)
                hornet.setStoragePos(!SharedData.isFull() ? SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) : (SharedData.greenIndex == 0 ? (SharedData.getGreenIndex() == -1 ? SharedData.getPurpleIndex() : SharedData.getGreenIndex()) : (SharedData.getPurpleIndex()) == -1 ? SharedData.getGreenIndex() : SharedData.getPurpleIndex()), !SharedData.isFull());
            launchingTemp = false;
        }else{hornet.startLaunchMotors(SharedData.shootFar);}

        // panels.getTelemetry().addData("key", value);
        // panels.getTelemetry().update();
        if ((pathState == 3 || pathState == 4 || pathState == 6 || pathState == 7)) {
            hornet.startIntake(true, 1);
        }
        else if(launching)
            hornet.startIntake(true,.1);
        else hornet.stopIntake();

        //SET STORAGE COLOR
        if(hornet.buttonPressed() && hornet.atSortTargetLenient() && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            //hornet.updateLED();
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
        if(launchingTemp && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.hammerAtLaunch() && !hornet.isLaunched()){
            hornet.launch();
            launchTimer.resetTimer();
        }

        if(launchTimer.getElapsedTimeSeconds() >= .25){
            if(hornet.hammerAtLaunch() && launchingTemp){
                launchTimer.resetTimer();
                hornet.resetHammer();
                SharedData.clearSlot(hornet.getSlotGoal());
            }
            else if(launchingTemp && hornet.isLaunched()){
                launchingTemp = false;
                timesLaunched++;
                if(timesLaunched == 2)
                    timesLaunched = 0;
                hornet.resetLaunch();
            }
        }
        telemetry.addData("Times Launched", timesLaunched);
        telemetry.addData("Green Index", SharedData.greenIndex);
//        telemetry.addData("Temp launch", launchingTemp);
//        telemetry.addData("launching", launching);
//        telemetry.addData("sort", hornet.atSortTarget() ? "at target" : "not at target");
//        telemetry.addData("launch motors", hornet.atTargetVelocity() ? "at velocity" : "not at velocity");
        telemetry.update();
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
                .addPath(new BezierLine(poses.LAUNCH_POSE, poses.ALIGN3_POSE))
                .setConstantHeadingInterpolation(poses.ALIGN3_POSE.getHeading())
                .setBrakingStrength(.5)
                //.setBrakingStart(.5)
                .build();
        five = f.pathBuilder()
                .addPath(new BezierLine(poses.ALIGN3_POSE, poses.PICKUP3_POSE))
                .setLinearHeadingInterpolation(poses.ALIGN3_POSE.getHeading(), poses.PICKUP3_POSE.getHeading())
                .build();
        six = f.pathBuilder()
                .addPath(new BezierLine(poses.PICKUP3_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.PICKUP3_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
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
                sendPose();
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > .25){
                    //go to align 1 pos
                    f.followPath(one, true);
                    setPathState(2);
                    sendPose();

                    launching = false;
                }
                //score 1
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 3)
                    launching = true;
                break;
            case 2:
                if (!f.isBusy()){
                    //pickup balls
                    f.followPath(two, true);
                    f.setMaxPower(.25);
                    setPathState(3);
                    sendPose();
                }
                break;
            case 3:
                if (!f.isBusy() || SharedData.isFull()){
                    //go to scoring pose
                    f.followPath(three, true);
                    f.setMaxPower(1);
                    setPathState(4);
                    sendPose();
                }
                break;
            case 4:
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > .25) {
                    //go to align 2
                    f.followPath(four, true);
                    setPathState(5);
                    sendPose();
                    launching = false;
                }
                //score 2
                else if (!f.isBusy()&& pathTimer.getElapsedTimeSeconds() > 3) {
                    launching = true;
                }
                break;
            case 5:
                if (!f.isBusy()){
                    //pickup 2
                    f.followPath(five , true);
                    f.setMaxPower(.25);
                    setPathState(6);
                    sendPose();
                }
                break;
            case 6:
                if (!f.isBusy() || SharedData.isFull()){
                    //move to score pose
                    f.followPath(six , true);
                    f.setMaxPower(1);
                    sendPose();
                    setPathState(7);
                }
                break;
            case 7:
                if (!f.isBusy() && SharedData.isEmpty() && launchTimer.getElapsedTimeSeconds() > .25 && opmodeTimer.getElapsedTimeSeconds() < 28.5){
                    // sends to final location
                    f.followPath(end);
                    setPathState(8);
                    sendPose();
                }
                //score 3
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 3){
                    launching = true;
                }
                break;
            case 8:
                if (!f.isBusy()){
                    setPathState(-1);
                    sendPose();
                    launching = false;
                }
                break;
        }
    }




    public int figureID (){
        result = limelight.getLatestResult();
        try{

            telemetry.addData("ID" , result.getFiducialResults().get(0).getFiducialId());
            return result.getFiducialResults().get(0).getFiducialId();
        }
        catch (Exception e){
            telemetry.addData("ID" , "not found");
            return -1;
        }
    }




    public void sendPose(){
        SharedData.toTeleopPose = f.getPose();
    }


}
