package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.Auto.PoseConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class MovementAuto extends OpMode {
    private Follower f;
    private Telemetry telemetryA;

    private PoseConstants poses = new PoseConstants();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path start, end;
    private PathChain one, two, three;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        f.update();
        autoPathUpdates();

        telemetryA.addData("test", true);
        telemetryA.update();
    }

    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("data");
        telemetryA.update();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        buildPaths();
    }

    @Override
    public void init_loop() {
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
        start = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(0, 0, 0)));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading(), poses.POSE_ONE.getHeading());

        one = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_ONE,poses.POSE_TWO))
                .setLinearHeadingInterpolation(poses.POSE_ONE.getHeading(), poses.POSE_TWO.getHeading())
                .build();
        two = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_TWO, poses.POSE_THREE ))
                .setLinearHeadingInterpolation(poses.POSE_TWO.getHeading(), poses.POSE_THREE.getHeading())
                .build();
        three = f.pathBuilder()
                .addPath(new BezierLine(poses.POSE_THREE, poses.POSE_FOUR ))
                .setLinearHeadingInterpolation(poses.POSE_THREE.getHeading(), poses.POSE_FOUR.getHeading())
                .build();
        end = new Path(new BezierLine(poses.POSE_FOUR, poses.END_POSE));
        end.setLinearHeadingInterpolation(poses.POSE_FOUR.heading, poses.END_POSE.heading());
    }


    public void autoPathUpdates() {
        switch (pathState) {
            case 0:
                f.followPath(start);
                setPathState(1);
                break;
            case 1:
                if (!f.isBusy()) {
                    f.followPath(one, true);
                    setPathState(2);
                }
            case 2:
                if (!f.isBusy()) {
                    f.followPath(two, true);
                    setPathState(3);
                }
            case 3:
                if (!f.isBusy()) {
                    f.followPath(three, true);
                    setPathState(4);
                }
            case 4:
                if (!f.isBusy()) {
                    f.followPath(end, true);
                    setPathState(5);
                }
            case 1:
                if (!f.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
}
