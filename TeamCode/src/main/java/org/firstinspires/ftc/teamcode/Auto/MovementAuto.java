package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path start, end, p, point;
    private PathChain one, two, three;

    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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
            if (detection.metadata!=null){
                return detection.id;
            }
        }
        return -1;
    }

    public void setPathState(int pState) {
        this.pathState = pState;
        pathTimer.resetTimer();
    }
    private void sort(boolean[] storage, int index)
    {
        if(storage[index])
            return;
        else
        {
            //do sorting
            sort(detect(),index);
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

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(poses.START_POSE);
        f.activateAllPIDFs();
        buildPaths();
    }

    @Override
    public void init_loop() {
        //detect

        telemetry.addData("April Tag", figureID());
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
        start = new Path(new BezierLine(poses.START_POSE, poses.POSE_ONE));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading(), poses.POSE_ONE.getHeading());

        p = new Path(new BezierLine(new Pose(0, 0, Math.PI / 2), new Pose(12, 12, Math.PI)));
        p.setLinearHeadingInterpolation(new Pose(0, 0, Math.PI / 2).getHeading(), new Pose(12, 12, Math.PI).getHeading());

        point = new Path( new BezierPoint(new Pose(0,0,Math.PI)));



        one = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_ONE, poses.POSE_TWO))
                .setLinearHeadingInterpolation(poses.POSE_ONE.getHeading(), poses.POSE_TWO.getHeading())
                .build();
        two = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_TWO, poses.POSE_THREE))
                .setLinearHeadingInterpolation(poses.POSE_TWO.getHeading(), poses.POSE_THREE.getHeading())
                .build();
        three = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_THREE, poses.POSE_FOUR))
                .setLinearHeadingInterpolation(poses.POSE_THREE.getHeading(), poses.POSE_FOUR.getHeading())
                .build();
        end = new Path(new BezierLine(poses.POSE_FOUR, poses.END_POSE));
        end.setLinearHeadingInterpolation(poses.POSE_FOUR.getHeading(), poses.END_POSE.getHeading());
    }


    public void autoPathUpdates() {


//        switch (pathState)  {
//            case 0:
//                f.turnToDegrees(180);
//
//                if ((Math.abs(f.getHeadingError())<Math.PI/40)&& (pathTimer.getElapsedTimeSeconds() > 4)) {
//                    setPathState(1);
//                }
//                    break;
//            case 1:
//                f.turnToDegrees(90);
//                if ((Math.abs(f.getHeadingError())<Math.PI/40) && (pathTimer.getElapsedTimeSeconds() > 4)) {
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                f.followPath(p);
//                if(!f.isBusy() && (pathTimer.getElapsedTimeSeconds() > 4)) {
//                setPathState(3);
//                }
//                break;
//            case 3:
//                if(!f.isBusy())
//                {
//                    setPathState(-1);
//                }
//                break;
//
//        }


        switch (pathState) {
            case 0:
                f.followPath(start);
                setPathState(1);
                break;
            case 1:
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(one, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(two, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(three, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    f.followPath(end, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!f.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
}
