package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class RedPoseConstants {
    public static Pose START_POSE = new Pose(48,0,Math.toRadians(270));
    public static Pose LAUNCH_POSE = new Pose(48,6,Math.toRadians(240));
    public static Pose ALIGN1_POSE = new Pose(31.5,27,Math.toRadians(180));
    public static Pose PICKUP1_POSE = new Pose(12,27, Math.toRadians(180));
    public static Pose ALIGN2_POSE = new Pose(31.5,51,Math.toRadians(180));
    public static Pose PICKUP2_POSE = new Pose(12, 51, Math.toRadians(180));
    public static Pose END_POSE = new Pose(12,12,Math.toRadians(240));
    public static Pose FIELD_OFFSET = new Pose(8.75,8.75);
    public static Pose CamOff = new Pose(-8,3.5);
    public static Pose parkPose = new Pose(29.625, 23.5,0);

}
