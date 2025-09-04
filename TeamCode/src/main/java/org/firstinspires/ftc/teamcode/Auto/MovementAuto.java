package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Point;

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

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path start,end;
    private PathChain one, two, three;

    public void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop()
    {
        f.update();
        autoPathUpdates();

        telemetryA.addData("test",true);
        telemetryA.update();
    }
    @Override
    public void init(){
        telemetryA = new MultipleTelemetry(this.telemetry,FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("data");
        telemetryA.update();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose(0,0,Math.toRadians(0)));

        buildPaths();
    }
    @Override
    public void init_loop(){}

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){}


    public void buildPaths(){
        start = new Path(new BezierLine(new Pose(0,0,0), new Pose(0,0,0)));
    }

    public void autoPathUpdates()
    {

    }
}
