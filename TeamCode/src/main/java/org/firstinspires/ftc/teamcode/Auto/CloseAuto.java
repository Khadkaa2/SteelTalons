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

//@Autonomous (name = "Auto CLOSE")
public class CloseAuto extends OpMode {

    private Robot hornet = new Robot();
    private Follower f = null;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    private int pathState;
    private Timer pathTimer,opmodeTimer,launchTimer,detectColorTimer, launchPauseTimer;
    private PoseConstantsClose posesClose = new PoseConstantsClose();
    Pose currentPose = null;
    private Path start, end;
    private PathChain one, two, three, four, five, six;
    int index;
    private Limelight3A limelight;
    boolean launching = false , launchingTemp = false;
    private LLResult result;
    int timesLaunched = 0;
    boolean firstPaused = false;

    @Override
    public void init() {
        hornet.initialize(this.hardwareMap);
        SharedData.reset();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        launchTimer = new Timer();
        launchPauseTimer = new Timer();
        opmodeTimer.resetTimer();
        launchPauseTimer.resetTimer();
        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(posesClose.START_POSE);
        buildPaths();
        hornet.disableLED();
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

        telemetry.update();
    }

    @Override
    public void start(){
        sendPose();
        opmodeTimer.resetTimer();
        limelight.stop();
        setPathState(0);
    }
    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();

        if(!launching) {
            hornet.resetHammer();
            hornet.resetLaunch();
            hornet.stopLaunchMotors();
            hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , !SharedData.isFull());
            launchingTemp = false;
        }else{hornet.startLaunchMotors(false);}

        // panels.getTelemetry().addData("key", value);
        // panels.getTelemetry().update();
        if ((pathState == 3 || pathState == 4 || pathState == 6 || pathState == 7) || launching) {
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

        if(launchingTemp && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.hammerAtLaunch() && !hornet.isLaunched() && launchPauseTimer.getElapsedTimeSeconds() > .5){
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
                hornet.resetLaunch();
                launchPauseTimer.resetTimer();
            }
        }

        telemetry.addData("Temp launch", launchingTemp);
        telemetry.addData("launching", launching);
        telemetry.addData("sort", hornet.atSortTarget() ? "at target" : "not at target");
        telemetry.addData("launch motors", hornet.atTargetVelocity() ? "at velocity" : "not at velocity");
    }

    public void stop(){
        sendPose();
    }

    public void setPathState(int state){
        this.pathState = state;
        pathTimer.resetTimer();
    }


    public void buildPaths(){
        start = new Path( new BezierLine(posesClose.START_POSE , posesClose.LAUNCH_POSE));
        start.setLinearHeadingInterpolation(posesClose.START_POSE.getHeading() , posesClose.LAUNCH_POSE.getHeading());

        one = f.pathBuilder()
                .addPath(new BezierLine( posesClose.LAUNCH_POSE , posesClose.ALIGN1_POSE))
                .setConstantHeadingInterpolation(posesClose.ALIGN1_POSE.getHeading())
                .build();
        two = f.pathBuilder()
                .addPath(new BezierLine( posesClose.ALIGN1_POSE , posesClose.PICKUP1_POSE))
                .setLinearHeadingInterpolation( posesClose.ALIGN1_POSE.getHeading() , posesClose.PICKUP1_POSE.getHeading())
                .build();
        three = f.pathBuilder()
                .addPath(new BezierLine(posesClose.PICKUP1_POSE , posesClose.LAUNCH_POSE ))
                .setLinearHeadingInterpolation(posesClose.PICKUP1_POSE.getHeading() , posesClose.LAUNCH_POSE.getHeading())
                .build();
        four = f.pathBuilder()
                .addPath(new BezierLine(posesClose.LAUNCH_POSE, posesClose.ALIGN2_POSE))
                .setConstantHeadingInterpolation(posesClose.ALIGN2_POSE.getHeading())
                .build();
        five = f.pathBuilder()
                .addPath(new BezierLine(posesClose.ALIGN2_POSE, posesClose.PICKUP2_POSE))
                .setLinearHeadingInterpolation(posesClose.ALIGN2_POSE.getHeading(), posesClose.PICKUP2_POSE.getHeading())
                .build();
        six = f.pathBuilder()
                .addPath(new BezierLine(posesClose.PICKUP2_POSE, posesClose.LAUNCH_POSE))
                .setLinearHeadingInterpolation(posesClose.PICKUP2_POSE.getHeading(), posesClose.LAUNCH_POSE.getHeading())
                .build();
        end = new Path(new BezierLine(posesClose.LAUNCH_POSE, posesClose.END_POSE));
        end.setLinearHeadingInterpolation(posesClose.LAUNCH_POSE.getHeading(), posesClose.END_POSE.getHeading());
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
                if (!f.isBusy() && SharedData.isEmpty()){
                    //go to align 1 pos
                    f.followPath(one, true);
                    setPathState(2);
                    sendPose();
                    launching = false;
                    firstPaused = false;
                }
                //score 1
                else if (!f.isBusy()) {
                    launching = true;
                    if (!firstPaused)
                    {
                        launchPauseTimer.resetTimer();
                        firstPaused = true;
                    }
                }
                break;
            case 2:
                if (!f.isBusy()){
                    //pickup balls
                    f.followPath(two, true);
                    f.setMaxPower(.15);
                    setPathState(3);
                    sendPose();
                }
                break;
            case 3:
                if (!f.isBusy()){
                    //go to scoring pose
                    f.followPath(three, true);
                    f.setMaxPower(1);
                    setPathState(4);
                    sendPose();
                }
                break;
            case 4:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    //go to align 2
                    f.followPath(four, true);
                    setPathState(5);
                    sendPose();
                    launching = false;
                    firstPaused = false;
                }
                //score 2
                else if (!f.isBusy()) {
                    launching = true;
                    if (!firstPaused)
                    {
                        launchPauseTimer.resetTimer();
                        firstPaused = true;
                    }


                }
                break;
            case 5:
                if (!f.isBusy()){
                    //pickup 2
                    f.followPath(five , true);
                    f.setMaxPower(.15);
                    setPathState(6);
                    sendPose();
                }
                break;
            case 6:
                if (!f.isBusy()){
                    //move to score pose
                    f.followPath(six , true);
                    f.setMaxPower(1);
                    sendPose();
                    setPathState(7);
                }
                break;
            case 7:
                if (!f.isBusy() && SharedData.isEmpty()){
                    // sends to final location
                    f.followPath(end);
                    setPathState(8);
                    sendPose();
                }
                //score 3
                else if (!f.isBusy()){
                    launching = true;
                    if (!firstPaused)
                    {
                        launchPauseTimer.resetTimer();
                        firstPaused = true;
                    }
                }
                break;
            case 8:
                if (!f.isBusy()){
                    setPathState(-1);
                    sendPose();
                    launching = false;
                    firstPaused = false;
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
