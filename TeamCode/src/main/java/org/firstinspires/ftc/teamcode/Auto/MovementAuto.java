package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class MovementAuto extends OpMode {
    private Follower f;
    //private Telemetry telemetryA;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private PoseConstants poses = new PoseConstants();

    private Timer pathTimer, actionTimer, opmodeTimer, sortTimer;
    private int pathState;

    private Path start, end, p, point;
    private PathChain one, two, three, four, five, six;

    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private boolean sorting = false;

    int index;

    CRServo intakeServo = null;

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

    public static int figureID(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 21||detection.id == 22||detection.id == 23)
                return detection.id;
        }
        return -1;
    }

    public static Pose3D robotPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 20||detection.id == 24)
                return detection.robotPose;
        }
        return null;
    }

    public void setPathState(int pState) {
        this.pathState = pState;
        pathTimer.resetTimer();
    }
    private void sort(boolean[] storage)
    {

        if(!sorting) {
            if (storage[index]) {
                sorting = false;
            }
            else {
                sorting = true;
                sortTimer.resetTimer();
            }
        }
    }
    private boolean[] detect()
    {
        boolean[] order = new boolean[3];
        //color sensor stuff
        return order;
    }

    @Override
    public void loop() {
        f.update();
        autoPathUpdates();


        panels.getTelemetry().addData("Path State", pathState);
        panels.getTelemetry().addData("Turning", f.isTurning());
        panels.getTelemetry().addData("headingError", f.getHeadingError());
        panels.getTelemetry().addData("heading", f.getPose().getHeading());
        panels.getTelemetry().addData("x", f.getPose().getX());
        panels.getTelemetry().addData("y", f.getPose().getY());
        panels.getTelemetry().update();

        if(sorting)
        {
            //set servo pos here
        }
        // #5 is holder -> time it takes to sort one ball
        if(sortTimer.getElapsedTimeSeconds()>5) {
        sort(detect());
        }
        telemetry.addData("Path State", pathState);
        Pose3D currentPose = robotPose();
        if (currentPose!=null){
            telemetry.addData("X",currentPose.getPosition().x);
            telemetry.addData("Y",currentPose.getPosition().y);
            telemetry.addData("H",currentPose.getOrientation());
        }
        if(pathState == 3||pathState == 6)
            intakeServo.setPower(1);
        else
            intakeServo.setPower(0);

    }

    @Override
    public void init() {
        SharedData.reset();
        initAprilTag();
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("data");
//        telemetryA.update();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        sortTimer = new Timer();


        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(poses.START_POSE);
        f.activateAllPIDFs();
        buildPaths();

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setPower(0);
        index = -1;
    }

    @Override
    public void init_loop() {
        //detect
        int ID = figureID();

        if (ID == 21) {
            index = 0;
        } else if (ID == 22) {
            index = 1;
        } else if (ID == 23) index = 2;
        telemetry.addData("Green Index", index );
        SharedData.greenIndex = index;
        Pose3D currentPose = robotPose();
        if (currentPose!=null){

            telemetry.addData("X",currentPose.getPosition().x);
            telemetry.addData("Y",currentPose.getPosition().y);
            telemetry.addData("H",currentPose.getOrientation().getYaw());

        }
        telemetry.addData("test","test");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }


    public void buildPaths() {
        start = new Path(new BezierLine(poses.START_POSE, poses.LAUNCH_POSE));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading(), poses.LAUNCH_POSE.getHeading());


//        p = new Path(new BezierLine(new Pose(0, 0, Math.PI / 2), new Pose(12, 12, Math.PI)));
//        p.setLinearHeadingInterpolation(new Pose(0, 0, Math.PI / 2).getHeading(), new Pose(12, 12, Math.PI).getHeading());
//
//        point = new Path( new BezierPoint(new Pose(0,0,Math.PI)));
//        point.setConstantHeadingInterpolation(Math.PI);


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


    public void autoPathUpdates() {


        switch (pathState) {
            case 0:
                //Score 1
                f.followPath(start);
                setPathState(1);
                break;
            case 1:
                //Align 1
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(one, true);
                    setPathState(2);

                }
                break;
            case 2:
                //Pickup 1
                if (!f.isBusy()) {
                    f.followPath(two, true);
                    f.setMaxPower(.5);
                    setPathState(3);

                }
                break;
            case 3:
                //Score 2
                if (!f.isBusy()) {
                    f.followPath(three, true);
                    f.setMaxPower(1);
                    setPathState(4);

                }
                break;
            case 4:
                //Align 2
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(four, true);
                    setPathState(5);


                }
                break;
            case 5:
                //Pickup 2
                if (!f.isBusy()) {
                    f.followPath(five, true);
                    f.setMaxPower(.5);
                    setPathState(6);
                }
                break;
            case 6:
                //Score 3
                if (!f.isBusy()) {
                    f.followPath(six, true);
                    f.setMaxPower(1);
                    setPathState(7);
                }
                break;
            case 7:
                //move to human area
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(end, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!f.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
}
