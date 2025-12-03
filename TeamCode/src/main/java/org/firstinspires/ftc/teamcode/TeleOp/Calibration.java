package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SharedData;

@TeleOp
public class Calibration extends LinearOpMode {

    DcMotorEx fan = null;
    boolean shootF;
    boolean startF;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        fan = hardwareMap.get(DcMotorEx.class, "fan");
        fan.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SharedData.reset();
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
            telemetry.addLine("B to set to red\nA to set to blue\nY to change start spot\nX to change shoot spot\n");
            telemetry.addData("Side", SharedData.red ? "red" : "blue");
            telemetry.addData("Shooting", SharedData.shootFar ? "far" : "close");
            telemetry.addData("Starting", SharedData.startFar ? "far" : "close");
            telemetry.update();
        }

    }
}
