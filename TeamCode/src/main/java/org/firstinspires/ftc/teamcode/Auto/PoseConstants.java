package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable
public class PoseConstants {
    public static Pose START_POSE = new Pose(0,-48,Math.toRadians(180));
    public static Pose POSE_ONE = new Pose(6,-48,Math.toRadians(150));
    public static Pose POSE_TWO = new Pose(27,-29,Math.toRadians(90));
    public static Pose POSE_THREE = new Pose(27,-12,Math.toRadians(90));
    public static Pose POSE_FOUR = new Pose(6,-48,Math.toRadians(150));
    public static Pose POSE_FIVE = new Pose(51,-29,Math.toRadians(90));
    public static Pose END_POSE = new Pose(51,-12,Math.toRadians(90));
    public static Pose FIELD_OFFSET = new Pose(8.75,8.75);
    public static Pose LAUNCH_POSE = new Pose(6,-48,Math.toRadians(150));
}
