package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//@TeleOp
public class LimeLightBP extends OpMode {


    private double heading;
    private double x;
    private double y;

    private GoBildaPinpointDriver odo;
    private double distance;
    private Limelight3A limelight = null;
    private IMU imu;
    private Pose3D pose = new Pose3D(new Position(DistanceUnit.INCH , 0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot rho = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(rho));
        limelight.pipelineSwitch(0);
        odo = hardwareMap.get(GoBildaPinpointDriver.class , "odo");
        heading = odo.getHeading(AngleUnit.DEGREES);
        x = odo.getPosX(DistanceUnit.INCH);
        y = odo.getPosY(DistanceUnit.INCH);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles heading = imu.getRobotYawPitchRollAngles(); /*new YawPitchRollAngles(AngleUnit.DEGREES, 0,0,0,0);*/
        limelight.updateRobotOrientation(heading.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            pose = result.getBotpose_MT2();
            //telemetry.addData("updated", limelight.updateRobotOrientation(heading.getYaw(AngleUnit.DEGREES)));
            telemetry.addData("target x" , result.getTx());
            telemetry.addData("target y" , result.getTy());
            telemetry.addData("target area" , result.getTa());
            try {
                telemetry.addData("detector result", result.getFiducialResults().get(0).getFiducialId());
            } catch (Exception e) {
                telemetry.addData("not", "working");
            }
            telemetry.addData("botPose" , pose.toString());
            telemetry.addData("yaw", pose.getOrientation().getYaw());
            telemetry.addData("Disance" , distance);

            telemetry.addData("distancex" , x);
            telemetry.addData("distancey" , y);
            telemetry.addData("odo heading" , heading);
            telemetry.update();

        }

    }
}
