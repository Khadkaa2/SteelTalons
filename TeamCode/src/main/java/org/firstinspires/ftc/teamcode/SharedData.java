package org.firstinspires.ftc.teamcode;
import androidx.lifecycle.viewmodel.CreationExtras;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class SharedData {
    public static int greenIndex = 0;
    public static Pose toTeleopPose;
    public static ColorSensed[] storage = {ColorSensed.GREEN, ColorSensed.PURPLE, ColorSensed.PURPLE};
    public static boolean red = true;

    public static void reset() {
        greenIndex = 0;
        storage[0] = ColorSensed.GREEN;
        storage[1] = ColorSensed.PURPLE;
        storage[2] = ColorSensed.PURPLE;
    }
    public static boolean isEmpty() {
        return (storage[0] == ColorSensed.NO_COLOR && storage[1] == ColorSensed.NO_COLOR && storage[2] == ColorSensed.NO_COLOR);
    }
    public static boolean isFull(){return (storage[0] != ColorSensed.NO_COLOR && storage[1] != ColorSensed.NO_COLOR && storage[2] != ColorSensed.NO_COLOR);}

    public static void emptyStorage() {
        storage[0] = ColorSensed.NO_COLOR;
        storage[1] = ColorSensed.NO_COLOR;
        storage[2] = ColorSensed.NO_COLOR;
    }
    public static int getGreenIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.GREEN)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.GREEN)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.GREEN)
            temp = 2;
        return temp == -1 ? getInconclusiveIndex() : temp;
    }
    public static int getPurpleIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.PURPLE)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.PURPLE)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.PURPLE)
            temp = 2;
        return temp == -1 ? getInconclusiveIndex() : temp;
    }
    public static int getInconclusiveIndex() {
        int temp = -1;
        if (SharedData.storage[0] == ColorSensed.INCONLUSIVE)
            temp = 0;
        else if (SharedData.storage[1] == ColorSensed.INCONLUSIVE)
            temp = 1;
        else if (SharedData.storage[2] == ColorSensed.INCONLUSIVE)
            temp = 2;
        return temp;

    }
    public static void clearSlot(int i) {storage[i] = ColorSensed.NO_COLOR;}

}