package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class SharedData {
    public static int greenIndex = 0;
    public static Pose toTeleopPose;
    public static ColorSensed[] storage = {ColorSensed.GREEN,ColorSensed.PURPLE,ColorSensed.PURPLE};
    public static void reset()
    {
        greenIndex = 0;
        storage[0] = ColorSensed.GREEN;
        storage[1] = ColorSensed.PURPLE;
        storage[2] = ColorSensed.PURPLE;
    }


}
