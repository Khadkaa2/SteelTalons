package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SharedData;
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

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.max(Math.abs(x), Math.abs(y)), rx);

            double frontLeftPower = (y + x + rx); /// (denominator * speedCoef);
            double backLeftPower = (y - x + rx); /// (denominator * speedCoef);
            double frontRightPower = (y - x - rx); /// (denominator * speedCoef);
            double backRightPower = (y + x - rx); /// (denominator * speedCoef);

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            telemetry.addData("Pattern", SharedData.greenIndex);


            if(gamepad1.right_bumper)
                intakeServo.setPower(1);
            else if (gamepad1.left_bumper) {
                intakeServo.setPower(-1);
            }
            else
                intakeServo.setPower(0);


            currentPose = robotPose();
            Pose rawPose = rawPose();
            Pose FTCPOSE = ftcPose();

            if (currentPose!= null)  telemetry.addData("Robotpose X,Y,H", Math.round(currentPose.getX()) + " " + currentPose.getY() + " " + currentPose.getHeading());

            if (rawPose != null)    telemetry.addData("Rawpose X,Y,H", rawPose.getX() + " " + rawPose.getY() + " " + rawPose.getHeading());

            if (FTCPOSE != null)    telemetry.addData("FTCPOSE X,Y,H", FTCPOSE.getX() + " " + FTCPOSE.getY() + " " + FTCPOSE.getHeading());


            telemetry.addData("Current test",currentPose == null);
            telemetry.addData("Raw test",rawPose == null);
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

    public  Pose robotPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 23||detection.id == 24){
                if(detection.metadata!= null)
                    return new Pose( detection.robotPose.getPosition().x , detection.robotPose.getPosition().y , detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                else
                    telemetry.addData("RobotMetadata", "null");
            }
        }

        return null;
    }

    public  Pose rawPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 23||detection.id == 24){
                if(detection.metadata!= null)
                    return new Pose( detection.rawPose.x , detection.rawPose.y , detection.rawPose.z);
                else
                    telemetry.addData("RawMetadata", "null");
            }
        }

        return null;
    }
    public  Pose ftcPose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if(detection.id == 23||detection.id == 24){
                if(detection.metadata!= null)
                    return new Pose( detection.ftcPose.x , detection.ftcPose.y , detection.ftcPose.yaw);
                else
                    telemetry.addData("FTC-Metadata", "null");
            }
        }

        return null;
    }


}


