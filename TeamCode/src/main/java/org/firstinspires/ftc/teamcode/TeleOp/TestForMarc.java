package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class TestForMarc extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;


    public void runOpMode (){
        leftBack = hardwareMap.get(DcMotor.class , "leftBack");
        rightBack = hardwareMap.get(DcMotor.class , "rightBack");
        leftFront = hardwareMap.get(DcMotor.class  , "leftFront");
        rightFront = hardwareMap.get(DcMotor.class  , "rightFront");



        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.right_trigger > 0) {
                leftBack.setPower(1);
            }



        }


    }



}
