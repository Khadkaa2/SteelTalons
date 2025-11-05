package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;
import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.panels.Panels;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Robot {
    private static DcMotorEx fan = null;
    private static DcMotorEx rightLaunch = null;
    private static DcMotorEx leftLaunch = null;
    private static CRServo intakeServo = null;
    private static CRServo feeder = null;
    private static ColorSensor entranceColor = null;
    private static DistanceSensor distanceSensor = null;
    private static TouchSensor touchSensor=null;
    int launchTargetVelocity;
    private static Servo test = null;
    int slotGoal;
    boolean launched;

     public Robot(HardwareMap hardwareMap){
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        entranceColor = hardwareMap.get(ColorSensor.class, "intakeColorSensor");
        fan = hardwareMap.get(DcMotorEx.class, "sortMotor");
        feeder = hardwareMap.get(CRServo.class, "feederServo");
        rightLaunch = hardwareMap.get(DcMotorEx.class, "rightLaunch");
        leftLaunch = hardwareMap.get(DcMotorEx.class, "leftLaunch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
         rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
         leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         feeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setStoragePos(int slot, boolean intake) {
        int ticks = 1426;
        int absolutePos = fan.getCurrentPosition();
        int relativePos = absolutePos % ticks;
        int rotationOffset = absolutePos-relativePos;
        slotGoal = slot;
        if (intake) {
            if (slot == 0) {
                if(relativePos <= ticks/2)
                    fan.setTargetPosition(rotationOffset);
                else
                    fan.setTargetPosition(rotationOffset + ticks);
            } else if (slot == 1) {
                if(relativePos <= 5*ticks/6)
                    fan.setTargetPosition(rotationOffset + ticks/3);
                else
                    fan.setTargetPosition(rotationOffset + 4*ticks/3);
            } else if (slot == 2) {
                if(relativePos >= ticks/6)
                    fan.setTargetPosition(rotationOffset + 2*ticks/3);
                else
                    fan.setTargetPosition(rotationOffset - ticks/3);

            }
        }
        else {
            if (slot == 0) {
                fan.setTargetPosition(rotationOffset + ticks/2);
            } else if (slot == 1) {
                if(relativePos >= ticks/3)
                    fan.setTargetPosition(rotationOffset + 5*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset - ticks/6);
            } else if (slot == 2) {
                if(relativePos >= 2*ticks/3)
                    fan.setTargetPosition(rotationOffset + 7*ticks/6);
                else
                    fan.setTargetPosition(rotationOffset + ticks/6);
            }
        }
    }

    public void startIntake(boolean in) {intakeServo.setPower(in ? 1 : -1);}
    public void stopIntake() {intakeServo.setPower(0);}

    public void startFeeder(boolean out) {feeder.setPower(out ? 1 : -1);}
    public void stopFeeder() {feeder.setPower(0);}

    public void startLaunchMotors(boolean far) {
         launchTargetVelocity = far ? 2500 : 1500;
         leftLaunch.setVelocity(launchTargetVelocity);
         rightLaunch.setVelocity(launchTargetVelocity);
    }

    public void stopLaunchMotors() {
         launchTargetVelocity = 0;
         leftLaunch.setVelocity(0);
         rightLaunch.setVelocity(0);
    }

    public boolean atTargetVelocity() {
        return leftLaunch.getVelocity() >= launchTargetVelocity - 50 && rightLaunch.getVelocity() >= launchTargetVelocity - 50;
    }

    public boolean atSortTarget() {return !fan.isBusy();}

    public int getSlotGoal() {return slotGoal;}

    public ColorSensed detectColor() {
         double saturation = JavaUtil.rgbToSaturation(entranceColor.red(), entranceColor.green(), entranceColor.blue());
         double hue = JavaUtil.rgbToHue(entranceColor.red(), entranceColor.green(), entranceColor.blue());
         return (hue > 170 && saturation < .5) ? ColorSensed.PURPLE : ((hue < 160 && saturation > .55) ? ColorSensed.GREEN : ColorSensed.INCONLUSIVE);
    }

    public void launch() {
         /*
         flap.setPosition(1);
          */
        launched = true;
    }
    public void resetFlap() {
        /*
        flap.setPosition(0);
         */
    }
    public void resetLaunch() {launched = false;}
    public boolean flapAtLaunch(){
         /*
         return flap.getPosition() == 1;
          */
        return false;
    }

    public boolean isLaunched(){return launched;}

    public boolean buttonPressed() {return touchSensor.isPressed();}




}
