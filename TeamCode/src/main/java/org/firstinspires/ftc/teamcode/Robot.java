package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private static DigitalChannel slotZeroGreen;
    private static DigitalChannel slotOneGreen;
    private static DigitalChannel slotTwoGreen;
    private static DigitalChannel slotZeroRed;
    private static DigitalChannel slotOneRed;
    private static DigitalChannel slotTwoRed;


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
        slotZeroGreen = hardwareMap.get(DigitalChannel.class, "slotZeroGreen");
        slotOneGreen = hardwareMap.get(DigitalChannel.class, "slotOneGreen");
        slotTwoGreen = hardwareMap.get(DigitalChannel.class, "slotTwoGreen");
         slotZeroRed = hardwareMap.get(DigitalChannel.class, "slotZeroRed");
         slotOneRed = hardwareMap.get(DigitalChannel.class, "slotOneRed");
         slotTwoRed = hardwareMap.get(DigitalChannel.class, "slotTwoRed");
         slotZeroGreen.setMode(DigitalChannel.Mode.OUTPUT);
         slotZeroRed.setMode(DigitalChannel.Mode.OUTPUT);
         slotOneGreen.setMode(DigitalChannel.Mode.OUTPUT);
         slotOneRed.setMode(DigitalChannel.Mode.OUTPUT);
         slotTwoGreen.setMode(DigitalChannel.Mode.OUTPUT);
         slotOneRed.setMode(DigitalChannel.Mode.OUTPUT);

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
         return (hue > 170 && saturation < .5) ? ColorSensed.PURPLE : ((hue < 160 && saturation > .55) ? ColorSensed.GREEN : ColorSensed.INCONCLUSIVE);
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

    public void updateLED() {
        slotZeroGreen.setState(SharedData.storage[0] == ColorSensed.GREEN || SharedData.storage[0] == ColorSensed.INCONCLUSIVE);
        slotZeroRed.setState(SharedData.storage[0] == ColorSensed.PURPLE || SharedData.storage[0] == ColorSensed.INCONCLUSIVE);
        slotOneGreen.setState(SharedData.storage[1] == ColorSensed.GREEN || SharedData.storage[1] == ColorSensed.INCONCLUSIVE);
        slotOneRed.setState(SharedData.storage[1] == ColorSensed.PURPLE || SharedData.storage[1] == ColorSensed.INCONCLUSIVE);
        slotTwoGreen.setState(SharedData.storage[2] == ColorSensed.GREEN || SharedData.storage[2] == ColorSensed.INCONCLUSIVE);
        slotTwoRed.setState(SharedData.storage[2] == ColorSensed.PURPLE || SharedData.storage[2] == ColorSensed.INCONCLUSIVE);

    }

    public void disableLED()
    {
        slotZeroGreen.setState(false);
        slotZeroRed.setState(false);
        slotOneGreen.setState(false);
        slotOneRed.setState(false);
        slotTwoGreen.setState(false);
        slotTwoRed.setState(false);

    }




}
