package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SharedData;

@Configurable
public class PoseConstants {
    public Pose START_POSE =  SharedData.red ? new Pose(48,0,Math.toRadians(270)) :  new Pose(96,0,Math.toRadians(270));
    public Pose LAUNCH_POSE = SharedData.red ? new Pose(48,6,Math.toRadians(240)) :  new Pose(96,6,Math.toRadians(300));
    public Pose ALIGN1_POSE = SharedData.red ? new Pose(31.5,27,Math.toRadians(180)) :  new Pose(112.5,27,Math.toRadians(0));
    public  Pose PICKUP1_POSE = SharedData.red ? new Pose(12,27, Math.toRadians(180)) :  new Pose(132,27, Math.toRadians(0));
    public Pose ALIGN2_POSE = SharedData.red ? new Pose(31.5,51,Math.toRadians(180)) :  new Pose(112.5,51,Math.toRadians(0));
    public Pose PICKUP2_POSE = SharedData.red ? new Pose(12, 51, Math.toRadians(180)) :  new Pose(132, 51, Math.toRadians(0));
    public Pose END_POSE = SharedData.red ? new Pose(12,12,Math.toRadians(240)) :  new Pose(132, 12,0);
//    public static Pose FIELD_OFFSET;
//    public static Pose CamOff;
    public Pose parkPose = SharedData.red ? new Pose(29.625, 23.5,0) :  new Pose(114.375,23.5,Math.toRadians(270));
}
