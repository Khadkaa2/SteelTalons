package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;


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

    public void runOpMode() throws InterruptedException{


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


        while (opModeIsActive()){

            if (gamepad1.y && !isSlowMode&& slowModeDelay <= 0) {
                isSlowMode = true; // Enable slow mode
                //debounceDelay();
                slowModeDelay = 20;
            } else if (gamepad1.y && isSlowMode&& slowModeDelay <= 0) {
                isSlowMode = false; // Disable slow mode
                //debounceDelay();
                slowModeDelay = 20;
            }
            slowModeDelay --;

            double y = -gamepad1.left_stick_y;  // Forward/Backward movement
            double x = gamepad1.left_stick_x;   // Strafing (left/right)
            double turn = gamepad1.right_stick_x;  // Turning (rotation)

// Apply non-linear scaling for joystick input (quadratic scaling for smoother low-speed control)
            y = Math.signum(y) * Math.pow(y, 2);  // Square the value for non-linear sensitivity
            x = Math.signum(x) * Math.pow(x, 2);  // Same for x-axis
            turn = Math.signum(turn) * Math.pow(turn, 2);  // And for turn

// Define speed factor based on whether the robot is in slow mode (D-pad up/down for y-axis, left/right for x-axis)
            boolean isSlowModeY = gamepad1.dpad_up || gamepad1.dpad_down;  // Slow mode for forward/backward (y-axis)
            boolean isSlowModeX = gamepad1.dpad_left || gamepad1.dpad_right;  // Slow mode for strafing (x-axis)

// Speed factors for the axes              hiiiii
            double speedFactorY = isSlowModeY ? 0.2 : 1;  // Apply slower speed in slow mode (0.2 for slow, 0.92 for normal)
            double speedFactorX = isSlowModeX ? 0.5 : 1;  // Apply slower speed for strafing (0.5 for slow, 0.92 for normal)

// Handle D-Pad input for strafing (left, right, up, down) with fixed slow mode
            if (gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down) {
                // Apply fixed speed for D-Pad movements
                if (gamepad1.dpad_left) {
                    x = -0.5;  // Strafe left
                    y = 0;     // Prevent forward/backward movement while strafing
                } else if (gamepad1.dpad_right) {
                    x = 0.5;   // Strafe right
                    y = 0;     // Prevent forward/backward movement while strafing
                } else if (gamepad1.dpad_up) {
                    y = 0.5;  // Move forward
                    x = 0;     // Prevent strafing while moving forward
                } else if (gamepad1.dpad_down) {
                    y = -0.5;   // Move backward
                    x = 0;     // Prevent strafing while moving backward
                }

                // Ensure that slow mode is active for D-pad actions
                speedFactorY = 0.2;  // Slow down forward/backward motion
                speedFactorX = 0.5;  // Slow down strafing motion
            }

// Denominator calculation for normalizing speed and to prevent division by zero
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 0.2);

// Calculate power for each wheel
            double frontLeftPower = ((y + x + turn) / denominator) * speedFactorY;
            double backLeftPower = ((y - x + turn) / denominator) * speedFactorY;
            double frontRightPower = ((y - x - turn) / denominator) * speedFactorY;
            double backRightPower = ((y + x - turn) / denominator) * speedFactorY;

// Set power to wheels
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);



            if (gamepad1.a){
                currentState = RobotState.INTAKE;
            }
            else if ( gamepad1.b){
                currentState = RobotState.ASCEND;
            }
            else if (gamepad1.x){
                currentState = RobotState.BASKET;
            }
            else if (gamepad1.right_trigger > 0.1){
                currentState = RobotState.SAMPLE;
            }
            else if (gamepad1.left_trigger > 0.1){
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



        }


    }

}
