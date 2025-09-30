package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.Auto.PoseConstants;

import com.pedropathing.paths.PathConstraints;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TeleOpControlled extends LinearOpMode {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private CRServo intakeServo;
    private PoseConstants poses =  new PoseConstants();

    private boolean automated = false;

    private PathChain toLaunch;

    private PanelsTelemetry panels = PanelsTelemetry.INSTANCE;

    private Follower f;

    private static AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Pose currentPose = null;

    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(new Pose());
        f.update();

        toLaunch = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(),poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(f.getPose().getHeading() , poses.LAUNCH_POSE.getHeading())
                .build();

        initAprilTag();

        waitForStart();

        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        f.startTeleopDrive();

        while (opModeIsActive()) {
            f.update();

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.max(Math.abs(x), Math.abs(y)), rx);

            if (!automated) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        true
                );
            }

//            double frontLeftPower = (y + x + rx); /// (denominator * speedCoef);
//            double backLeftPower = (y - x + rx); /// (denominator * speedCoef);
//            double frontRightPower = (y - x - rx); /// (denominator * speedCoef);
//            double backRightPower = (y + x - rx); /// (denominator * speedCoef);

//            frontLeft.setPower(frontLeftPower);
//            backLeft.setPower(backLeftPower);
//            frontRight.setPower(frontRightPower);
//            backRight.setPower(backRightPower);
            telemetry.addData("Pattern", SharedData.greenIndex);




            if (gamepad1.a){

                f.followPath(toLaunch);
                automated = true;

            }
            if (automated && (gamepad1.b || !f.isBusy())){
                f.startTeleopDrive();
                automated = false;
            }


            telemetry.addData("GOINGTOLAUNCH MAINLOOP", f.isBusy());

            /*maybe else breakPath so you have to hold start in order
              to actually do the entire path to allow for mid-path
              interupption in case there's anything in the way */
            if(gamepad1.right_bumper)
                intakeServo.setPower(1);
            else if (gamepad1.left_bumper) {
                intakeServo.setPower(-1);
            }
            else
                intakeServo.setPower(0);



            Pose FTCPOSE = ftcPose();



            if (FTCPOSE != null)    telemetry.addData("FTCPOSE X,Y,H", FTCPOSE.getX() + ", " + FTCPOSE.getY() + " " + FTCPOSE.getHeading());

            telemetry.addData("FOLLOWER X",f.getPose().getX());
            telemetry.addData("FOLLOWER Y",f.getPose().getY());
            telemetry.addData("FOLLOWER Heading",f.getPose().getHeading());

            telemetry.addData("FTC test",FTCPOSE == null);



            telemetry.update();

        }
    }
    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }


    public  Pose ftcPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 23||detection.id == 24){
                if(detection.metadata!= null)
                    return new Pose( detection.ftcPose.x , detection.ftcPose.y , detection.ftcPose.bearing);
                else
                    telemetry.addData("FTC-Metadata", "null");
            }
        }

        return null;
    }


}


