package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.bylazar.panels.Panels;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;



@TeleOp
public class MotorTest extends LinearOpMode {

    private Follower f;
    DcMotorEx frontLeft = null;
    DcMotorEx frontRight = null;
    DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;


    public void runOpMode() throws InterruptedException {

        f = Constants.createFollower(hardwareMap);


        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        waitForStart();
//        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
//        backRight.setDirection(DcMotorEx.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeIsActive()) {
            f.update();
            if (gamepad1.x) {
                frontLeft.setPower(.5);
            } else if (gamepad1.y) {
                frontRight.setPower(.5);
            } else if (gamepad1.b) {
                backLeft.setPower(.5);
            } else if (gamepad1.a) backRight.setPower(.5);
            else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            telemetry.addData("x pose", f.getPose().getX());
            telemetry.addData("y pose", f.getPose().getY());
            telemetry.addData("frontLeftPower", frontLeft.getPower());
            telemetry.addData("frontRightPower", frontRight.getPower());
            telemetry.addData("backLeftPower", backLeft.getPower());
            telemetry.addData("backRightPower", backRight.getPower());
            telemetry.update();

        }

    }
}
