package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SharedData;

@TeleOp
public class Calibration extends LinearOpMode {

    DcMotorEx fan = null;
    @Override
    public void runOpMode() throws InterruptedException {
        SharedData.red = !SharedData.red;
        waitForStart();
        fan = hardwareMap.get(DcMotorEx.class, "fan");
        fan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SharedData.reset();
        SharedData.emptyStorage();
        while(opModeIsActive())
        {
            if(gamepad1.a || gamepad2.a)
                SharedData.red = false;
            else if(gamepad1.b || gamepad2.b)
                SharedData.red = true;
            if(gamepad1.dpad_left || gamepad2.dpad_left)
                SharedData.emptyStorage();
            if(gamepad1.dpad_right || gamepad2.dpad_right)
                SharedData.reset();
            if(gamepad1.y || gamepad2.y)
                fan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("B to set to red\nA to set to blue\ndpad left to empty storage\ndpad right to reset SharedData\nY to reset fan encoder\n");
            telemetry.addData("side", SharedData.red ? "red" : "blue");
            telemetry.update();
        }

    }
}
