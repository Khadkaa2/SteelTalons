package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SharedData;

@TeleOp
public class Calibration extends LinearOpMode {

    DcMotorEx fan;
    @Override
    public void runOpMode() throws InterruptedException {
        SharedData.red = !SharedData.red;
        waitForStart();
        fan = null;
        fan = hardwareMap.get(DcMotorEx.class, "fan");
        fan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SharedData.reset();
        SharedData.emptyStorage();

    }
}
