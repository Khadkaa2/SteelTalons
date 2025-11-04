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

@TeleOp
public class NewTeleOp extends LinearOpMode{
    Robot hornet;
    Follower f = Constants.createFollower(hardwareMap);
    PoseConstants poses = new PoseConstants();
    boolean robotCentric;
    boolean autoMode;
    double speedMultiplier = 1;

    public void runOpMode()
    {
        hornet = new Robot(hardwareMap);
        hornet.setStoragePos(1,true);
        hornet.startIntake(true);
        hornet.stopIntake();
        hornet.startFeeder(true);
        hornet.stopFeeder();
        hornet.startLaunchMotors(true);
        hornet.stopLaunchMotors();
        hornet.atTargetVelocity();
        hornet.detectColor();

        waitForStart();
        while(opModeIsActive())
        {
            f.setTeleOpDrive(
                    -gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    robotCentric,
                    (SharedData.red || robotCentric) ? 0 : Math.toRadians(180)
            );

            if (autoMode) {autoMode();} else {manualMode();}


        }

    }
    public void autoMode()
    {
        hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , SharedData.storage[0] != ColorSensed.NO_COLOR || SharedData.storage[1] != ColorSensed.NO_COLOR || SharedData.storage[2] != ColorSensed.NO_COLOR);

        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            SharedData.storage[gamepad2.a ? 0 : (gamepad2.b ? 1 : (gamepad2.y ? 2 : -1))] = ColorSensed.NO_COLOR;
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.NO_COLOR;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.NO_COLOR;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.NO_COLOR;
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
        if(touchSensor.isPressed() && hornet.atSortTarget) && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            SharedData.storage[hornet.getSlotGoal] = hornet.detectColor();
        }
         */

    }
    public void manualMode()
    {
        if(gamepad2.a)
            hornet.setStoragePos(0, gamepad2.dpad_left);
        else if(gamepad2.b)
            hornet.setStoragePos(1, gamepad2.dpad_left);
        else if(gamepad2.y)
            hornet.setStoragePos(2, gamepad2.dpad_left);
    }
}
