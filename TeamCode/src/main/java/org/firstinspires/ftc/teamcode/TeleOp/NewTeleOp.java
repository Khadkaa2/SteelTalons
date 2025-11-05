package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.graphics.Color;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.FTCCoordinates;


import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;


import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "AAA TeleOp")
public class NewTeleOp extends LinearOpMode{
    Robot hornet;
    Follower f = Constants.createFollower(hardwareMap);
    PoseConstants poses = new PoseConstants();
    boolean robotCentric;
    boolean autoMode;
    double speedMultiplier = 1;
    boolean launching;
    boolean automated;
    private PathChain toLaunchSame, toPark, toLaunchCross;

    boolean slowMode;

    public void runOpMode()
    {
        hornet = new Robot(hardwareMap);
        createPaths();

        waitForStart();
        while(opModeIsActive())
        {
            if(!automated) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        robotCentric,
                        (SharedData.red || robotCentric) ? 0 : Math.toRadians(180)
                );
            }

            //Launch Spot 1
            if (gamepad1.a && !automated) {
                f.followPath(toLaunchSame);
                automated = true;
            }
            //Auto Pathing to Park
            else if (gamepad1.x && !automated) {
                f.followPath(toPark);
                automated = true;
            }
            //Launch Spot 2
            else if (gamepad1.b && !automated) {
                f.followPath(toLaunchCross);
                automated = true;
            }
            //exits automated pathing
            else if ((!gamepad1.a && !gamepad1.x && !gamepad1.b) && automated) {
                f.startTeleopDrive(true);
                automated = false;
            }


            if(slowMode != gamepad1.y && gamepad1.y)
                speedMultiplier = speedMultiplier == 1 ? .2 : 1;
            slowMode = gamepad1.y;

            if (autoMode) {autoMode();} else {manualMode();}


        }

    }
    public void autoMode() {
        if(!launching)
            hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , (SharedData.storage[0] != ColorSensed.NO_COLOR || SharedData.storage[1] != ColorSensed.NO_COLOR || SharedData.storage[2] != ColorSensed.NO_COLOR));

        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.clearSlot(0);
            else if(gamepad2.b)
                SharedData.clearSlot(1);
            else if(gamepad2.y)
                SharedData.clearSlot(2);
        }
        else if(gamepad2.right_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.GREEN;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.GREEN;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.GREEN;
        }
        else if(gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.PURPLE;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.PURPLE;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.PURPLE;
        }

        /*
        if(touchSensor.isPressed() && hornet.atSortTarget()) && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            SharedData.storage[hornet.getSlotGoal()] = hornet.detectColor();
        }
         */

        if(gamepad2.dpad_up && SharedData.getGreenIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getGreenIndex(), false);
            launching = true;
        }
        if(gamepad2.dpad_down && SharedData.getPurpleIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getPurpleIndex(), false);
            launching = true;
        }

        /*
        //If launching -> speed up launchMotors
        if(launching){hornet.startLaunchMotors(true);}

        //if ready to launch -> then launch
        if(launching && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.flapAtLaunch()){
            hornet.launch();
            launchTimer.resetTimer();
        }

        //if flap has had time to move...
        and flap is at launch position -> move flap back and clear storage slot
        and flap is at not launch position and it says its launching -> say its not launching
        if(launchTimer.getElapsedTimerSeconds() >= .25){
            if(hornet.flapAtLaunch()){
                launchTimer.resetTimer();
                hornet.resetFlap();
                SharedData.clear(hornet.getSlotGoal());
            }
            else if(launching && hornet.launched()){
                launching = false;
                hornet.resetLaunch();
            }
        }
         */

    }
    public void manualMode() {
        //sorter
        if(gamepad2.a)
            hornet.setStoragePos(0, !gamepad2.dpad_left);
        else if(gamepad2.b)
            hornet.setStoragePos(1, !gamepad2.dpad_left);
        else if(gamepad2.y)
            hornet.setStoragePos(2, !gamepad2.dpad_left);

        //launch
        if(gamepad2.right_trigger > .2)
            hornet.startLaunchMotors(true);
        else if(gamepad2.left_trigger > .2)
            hornet.startLaunchMotors(false);
        else
            hornet.stopLaunchMotors();

        //flap
        /*
        if(gamepad2.right_bumper)
            hornet.launch()
        if(gamepad2.left_bumper){
            hornet.resetFlap();
            hornet.resetLaunch();
        }
         */
    }
    public void createPaths() {
        toLaunchSame = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        toLaunchCross = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.teleOpLaunchPose))
                .setConstantHeadingInterpolation(poses.teleOpLaunchPose.getHeading())
                .build();

        toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.parkPose))
                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
                .build();
    }
}
