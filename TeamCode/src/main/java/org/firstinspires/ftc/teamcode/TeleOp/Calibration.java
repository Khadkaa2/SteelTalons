package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SharedData;

@TeleOp
public class Calibration extends LinearOpMode {

    DcMotor sortMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        SharedData.red = !SharedData.red;
        waitForStart();
        sortMotor = null;
        sortMotor = hardwareMap.get(DcMotorEx.class, "sortMotor");
        sortMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SharedData.reset();
        SharedData.emptyStorage();

    }
}
