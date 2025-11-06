package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp
public class LimeLightBP extends OpMode {

    private Limelight3A limelight = null;
    private IMU imu;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot rho = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(rho));
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles heading = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(heading.getYaw());
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            Pose3D pose = result.getBotpose_MT2();
            telemetry.addData("target x" , result.getTx());
            telemetry.addData("target y" , result.getTy());
            telemetry.addData("target h" , result.getTa());
            telemetry.addData("botPose" , pose.toString());
            telemetry.addData("yaw", pose.getOrientation().getYaw());

            telemetry.update();

        }

    }
}
