package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.bylazar.telemetry.PanelsTelemetry;


@TeleOp
public class Teleop extends LinearOpMode {

    private enum RobotState {
        INIT,
        INTAKE,
        BASKET,
        SAMPLE,
        SPECIMEN,
        ASCEND
    }

    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    DcMotorEx backRight = null;
    boolean isSlowMode;
    int slowModeDelay;
    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private Follower f;

    public void runOpMode() throws InterruptedException {

        f = Constants.createFollower(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        waitForStart();
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RobotState currentState = RobotState.INIT;


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.max(Math.abs(x), Math.abs(y)), rx);

            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
                denominator = denominator * 2;
            }

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            if (gamepad1.a) {
                currentState = RobotState.INTAKE;
            } else if (gamepad1.b) {
                currentState = RobotState.ASCEND;
            } else if (gamepad1.x) {
                currentState = RobotState.BASKET;
            } else if (gamepad1.right_trigger > 0.1) {
                currentState = RobotState.SAMPLE;
            } else if (gamepad1.left_trigger > 0.1) {
                currentState = RobotState.SPECIMEN;
            }


            //            switch (currentState){
//                case INIT:
//                    telemetry.addData("State", "INIT");
//                    telemetry.update();
//                    break;
//
//                case INTAKE:
//                    telemetry.addData("State", "INTAKE");
//                    telemetry.update();
//                    break;
//
//                case ASCEND:
//                    telemetry.addData("State", "ASCEND");
//                    telemetry.update();
//                    break;
//
//                case SAMPLE:
//                    telemetry.addData("State", "SAMPLE");
//                    telemetry.update();
//                    break;
//
//                case SPECIMEN:
//                    telemetry.addData("State", "SPECIMEN");
//                    telemetry.update();
//                    break;
//
//                case BASKET:
//                    telemetry.addData("State", "BASKET");
//                    telemetry.update();
//                    break;
//            }

            f.update();
            panels.getTelemetry().addData("x", f.getPose().getX());
            panels.getTelemetry().addData("y", f.getPose().getY());
            panels.getTelemetry().update();
        }


    }
}


