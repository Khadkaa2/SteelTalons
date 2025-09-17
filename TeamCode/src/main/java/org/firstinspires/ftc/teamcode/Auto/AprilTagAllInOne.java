package org.firstinspires.ftc.teamcode.Auto;


import androidx.lifecycle.ViewModelStoreOwner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class AprilTagAllInOne extends LinearOpMode {

    private static AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();

        String pattern = "";

        waitForStart();




        while (opModeIsActive()){
            int ID = figureID();
            if (ID == 21) {
                pattern = "GREEN-PURPLE-PURPLE";
            } else if (ID == 22) {
                pattern = "PURPLE-GREEN-PURPLE";
            } else if (ID == 23) pattern = "PURPLE-PURPLE-GREEN";
            else if (ID == -1) pattern = "NOT-DETECTED";

            telemetry.addData("COLOR", pattern);
            telemetry.addData("AprilTag ID", ID);
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

    public static int figureID(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections){
            if (detection.metadata!=null){
                return detection.id;
            }
            else return -1;
        }
        return -1;
    }

}
