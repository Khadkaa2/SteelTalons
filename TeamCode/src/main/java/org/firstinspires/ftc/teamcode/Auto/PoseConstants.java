package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SharedData;

@Configurable
public class PoseConstants {
    public Pose START_POSE =  !SharedData.red ? new Pose(48,0,Math.toRadians(270)) :  new Pose(96,0,Math.toRadians(270));
    public Pose LAUNCH_POSE = !SharedData.red ? new Pose(48,6,Math.toRadians(293)) :  new Pose(96,6,Math.toRadians(243));
    public Pose ALIGN1_POSE = !SharedData.red ? new Pose(37,26,Math.toRadians(180)) :  new Pose(106,26,Math.toRadians(0));
    public  Pose PICKUP1_POSE = !SharedData.red ? new Pose(9,26, Math.toRadians(180)) :  new Pose(135,26, Math.toRadians(0));
    public Pose ALIGN2_POSE = !SharedData.red ? new Pose(37,50,Math.toRadians(180)) :  new Pose(106,50,Math.toRadians(0));
    public Pose PICKUP2_POSE = !SharedData.red ? new Pose(9, 50, Math.toRadians(180)) :  new Pose(135, 50, Math.toRadians(0));
    public Pose END_POSE = !SharedData.red ? new Pose(12,12,Math.toRadians(240)) :  new Pose(132, 12,0);
//    public static Pose FIELD_OFFSET;
//    public static Pose CamOff;
    public Pose parkPose = !SharedData.red ? new Pose(29.625, 23.5,0) :  new Pose(114.375,23.5,Math.toRadians(270));
    public Pose teleOpLaunchPose = !SharedData.red ? new Pose(96,6, Math.toRadians(243)) : new Pose(48,6,Math.toRadians(297));
}
