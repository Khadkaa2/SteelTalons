package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class SharedData {
    public static int greenIndex = 0;
    public static Pose toTeleopPose;
    public static void reset()
    {

        greenIndex = -1;
    }
}
