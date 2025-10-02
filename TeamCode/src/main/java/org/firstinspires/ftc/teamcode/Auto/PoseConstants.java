package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class PoseConstants {
    public static Pose START_POSE = new Pose(0,-48,Math.toRadians(180));
    public static Pose LAUNCH_POSE = new Pose(6,-48,Math.toRadians(150));
    public static Pose ALIGN1_POSE = new Pose(27,-31.5,Math.toRadians(90));
    public static Pose PICKUP1_POSE = new Pose(27,-12, Math.toRadians(90));
    public static Pose ALIGN2_POSE = new Pose(51,-31.5,Math.toRadians(90));
    public static Pose PICKUP2_POSE = new Pose(51, -12, Math.toRadians(90));
    public static Pose END_POSE = new Pose(12,-12,Math.toRadians(180));
    public static Pose FIELD_OFFSET = new Pose(8.75,8.75);
    public static Pose CamOff = new Pose(-8,3.5);
    public static Pose parkPose = new Pose(23.5, -29.625,0);

}
