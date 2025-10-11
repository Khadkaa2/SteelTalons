package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class BluePoseConstants {
    public static Pose START_POSE = new Pose(96,0,Math.toRadians(270));
    public static Pose LAUNCH_POSE = new Pose(96,6,Math.toRadians(300));
    public static Pose ALIGN1_POSE = new Pose(112.5,27,Math.toRadians(0));
    public static Pose PICKUP1_POSE = new Pose(132,27, Math.toRadians(0));
    public static Pose ALIGN2_POSE = new Pose(112.5,51,Math.toRadians(0));
    public static Pose PICKUP2_POSE = new Pose(132, 51, Math.toRadians(0));
    public static Pose END_POSE = new Pose(132,12,Math.toRadians(300));
    public static Pose FIELD_OFFSET = new Pose(8.75,8.75);

    //check CamOff
    public static Pose CamOff = new Pose(-8,3.5);
    public static Pose parkPose = new Pose(144.375, 23.5,0);

}
