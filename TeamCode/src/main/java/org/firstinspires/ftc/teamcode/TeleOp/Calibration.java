package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.General.SharedData;

@TeleOp
public class Calibration extends LinearOpMode {

    DcMotorEx fan = null;
    boolean shootF;
    boolean startF;
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
            else if((gamepad1.x || gamepad2.x) && (gamepad1.x || gamepad2.x) != shootF)
                SharedData.shootFar = !SharedData.shootFar;
            else if((gamepad1.y || gamepad2.y) && (gamepad1.y || gamepad2.y) != startF)
                SharedData.startFar = !SharedData.startFar;
            shootF = gamepad1.x || gamepad2.x;
            startF = gamepad1.y || gamepad2.y;
            telemetry.addLine("B to set to red\nA to set to blue\ndpad left to empty storage\ndpad right to reset SharedData\nY to reset fan encoder\n");
            telemetry.addData("Side", SharedData.red ? "red" : "blue");
            telemetry.addData("Shooting", SharedData.shootFar ? "far" : "close");
            telemetry.addData("Starting", SharedData.startFar ? "far" : "close");
            telemetry.update();
        }

    }
}
