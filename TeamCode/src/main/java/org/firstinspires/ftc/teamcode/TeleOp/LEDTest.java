package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class LEDTest extends LinearOpMode {
    private Robot hornet = new Robot();
//    private static LED slotZeroGreen;
//    private static LED slotOneGreen;
//    private static LED slotTwoGreen;
//    private static LED slotZeroRed;
//    private static LED slotOneRed;
//    private static LED slotTwoRed;

    @Override
    public void runOpMode() throws InterruptedException {
            hornet.initialize(hardwareMap);
//        slotZeroGreen = hardwareMap.get(LED.class, "slotZeroGreen");
//        slotZeroRed = hardwareMap.get(LED.class, "slotZeroRed");
//        slotOneGreen = hardwareMap.get(LED.class, "slotOneGreen");
//        slotOneRed = hardwareMap.get(LED.class, "slotOneRed");
//        slotTwoGreen = hardwareMap.get(LED.class, "slotTwoGreen");
//        slotTwoRed = hardwareMap.get(LED.class, "slotTwoRed");


        waitForStart();
        while(opModeIsActive()) {

//            slotZeroGreen.off();
//            slotZeroRed.on();
//            slotOneGreen.off();
//            slotOneRed.on();
//            slotTwoGreen.off();
//            slotTwoRed.on();
//            sleep(5000);
//            slotZeroGreen.on();
//            slotZeroRed.off();
//            slotOneGreen.on();
//            slotOneRed.off();
//            slotTwoGreen.on();
//            slotTwoRed.off();
//            sleep(5000);

            hornet.updateLED();
            sleep(1000);
            hornet.disableLED();
            sleep(1000);
        }

    }
}
