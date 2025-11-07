package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class Robot {
    public static DcMotorEx fan = null;
    public static DcMotorEx rightLaunch = null;
    public static DcMotorEx leftLaunch = null;
    public static CRServo intakeServo = null;
    public static Servo hammer = null;
    private static ColorSensor colorRight = null;
    private static ColorSensor colorLeft = null;
    private static TouchSensor touchSensor=null;
    private Limelight3A limelight = null;
    private LLResult result = null;
    private static LED slotZeroGreen;
    private static LED slotOneGreen;
    private static LED slotTwoGreen;
    private static LED slotZeroRed;
    private static LED slotOneRed;
    private static LED slotTwoRed;


    int launchTargetVelocity;
    private static Servo test = null;
    int slotGoal;
    boolean launched;


     public void initialize(HardwareMap hwMp){
        intakeServo = hwMp.get(CRServo.class, "intakeServo");
        colorRight = hwMp.get(ColorSensor.class, "colorRight");
        fan = hwMp.get(DcMotorEx.class, "fan");
        hammer = hwMp.get(Servo.class, "hammer");
        rightLaunch = hwMp.get(DcMotorEx.class, "rightLaunch");
        leftLaunch = hwMp.get(DcMotorEx.class, "leftLaunch");
        colorLeft = hwMp.get(ColorSensor.class, "colorLeft");
        touchSensor = hwMp.get(TouchSensor.class, "touchSensor");

        slotZeroGreen = hwMp.get(LED.class, "slotZeroGreen");
        slotOneGreen = hwMp.get(LED.class, "slotOneGreen");
        slotTwoGreen = hwMp.get(LED.class, "slotTwoGreen");
         slotZeroRed = hwMp.get(LED.class, "slotZeroRed");
         slotOneRed = hwMp.get(LED.class, "slotOneRed");
         slotTwoRed = hwMp.get(LED.class, "slotTwoRed");

         rightLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
         leftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         leftLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         rightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         rightLaunch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         fan.setTargetPosition(0);
         fan.setTargetPositionTolerance(10);
         fan.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         fan.setPower(1);

         limelight = hwMp.get(Limelight3A.class, "limelight");
         LLResult result = limelight.getLatestResult();
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

    public void startLaunchMotors(boolean far) {
         launchTargetVelocity = far ? 2150 : 1500;
         leftLaunch.setVelocity(launchTargetVelocity);
         rightLaunch.setVelocity(launchTargetVelocity);
    }

    public int getLaunchTargetVelocity(){
         return launchTargetVelocity;
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
         double saturation = JavaUtil.rgbToSaturation(colorRight.red(), colorRight.green(), colorRight.blue());
         double hue = JavaUtil.rgbToHue(colorRight.red(), colorRight.green(), colorRight.blue());
         return (hue > 170 && saturation < .5) ? ColorSensed.PURPLE : ((hue < 160 && saturation > .55) ? ColorSensed.GREEN : ColorSensed.INCONCLUSIVE);
    }

    public void launch() {
         hammer.setPosition(1);
        launched = true;
    }
    public void resetFlap() {
        hammer.setPosition(0);
    }
    public void resetLaunch() {launched = false;}
    public boolean flapAtLaunch(){
         return hammer.getPosition() == 1;
    }

    public boolean isLaunched(){return launched;}

    public boolean buttonPressed() {return touchSensor.isPressed();}

    public void updateLED() {

         if(SharedData.storage[0] == ColorSensed.PURPLE || SharedData.storage[0] == ColorSensed.INCONCLUSIVE)
            slotZeroGreen.on();
         else slotZeroGreen.off();
        if(SharedData.storage[0] == ColorSensed.GREEN || SharedData.storage[0] == ColorSensed.INCONCLUSIVE)
            slotZeroRed.on();
        else slotZeroRed.off();
        if(SharedData.storage[1] == ColorSensed.PURPLE || SharedData.storage[1] == ColorSensed.INCONCLUSIVE)
            slotOneGreen.on();
        else slotOneGreen.off();
        if(SharedData.storage[1] == ColorSensed.GREEN || SharedData.storage[1] == ColorSensed.INCONCLUSIVE)
            slotOneRed.on();
        else slotOneRed.off();
        if(SharedData.storage[2] == ColorSensed.PURPLE || SharedData.storage[2] == ColorSensed.INCONCLUSIVE)
            slotTwoGreen.on();
        else slotTwoGreen.off();
        if(SharedData.storage[2] == ColorSensed.GREEN || SharedData.storage[2] == ColorSensed.INCONCLUSIVE)
            slotTwoRed.on();
        else slotTwoRed.off();

    }

    public void disableLED()
    {
        slotZeroGreen.off();
        slotZeroRed.off();
        slotOneGreen.off();
        slotOneRed.off();
        slotTwoGreen.off();
        slotTwoRed.off();

    }

    public void getTargetArea(){
         if (result.isValid()){

         }
    }

    public void getResult(){

        if (this.result.isValid() && result != null){

        }
    }







}
