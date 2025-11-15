package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SharedData;

@Configurable
public class PoseConstants {
    public Pose START_POSE =  !SharedData.red ? (SharedData.startFar ? new Pose(56.75,8.75,Math.toRadians(270)) : new Pose(27.85, 125.85, Math.toRadians(190.75))) :  (SharedData.startFar ? new Pose(87.25,8.75,Math.toRadians(270)) : new Pose(118.55, 125.25, Math.toRadians(350)));
    public Pose LAUNCH_POSE = !SharedData.red ? (SharedData.shootFar ? new Pose(56.75,14.75,Math.toRadians(294)) : new Pose(56.75,80.75,Math.toRadians(315))) :  (SharedData.shootFar ? new Pose(87.25,14.75,Math.toRadians(246)) : new Pose(87.25,80.75,Math.toRadians(225)));
    public Pose ALIGN1_POSE = !SharedData.red ? (SharedData.shootFar ? new Pose(45.75,34.75,Math.toRadians(180)) : new Pose(45.75,82.75,Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(98.25,34.75,Math.toRadians(0)) : new Pose(98.25,82.75,Math.toRadians(0)));
    public  Pose PICKUP1_POSE = !SharedData.red ? (SharedData.shootFar ? new Pose(17.75,34.75, Math.toRadians(180)) : new Pose(17.75,82.75, Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(126.25,34.75, Math.toRadians(0)) : new Pose(126.25,82.75, Math.toRadians(0)));
    public Pose ALIGN2_POSE = !SharedData.red ? new Pose(45.75,58.75,Math.toRadians(180)) :  new Pose(98.25,58.75,Math.toRadians(0));
    public Pose PICKUP2_POSE = !SharedData.red ? new Pose(17.75, 58.75, Math.toRadians(180)) :  new Pose(126.25, 58.75, Math.toRadians(0));
    public Pose END_POSE = !SharedData.red ? (SharedData.shootFar ? new Pose(20.75,20.75,Math.toRadians(240)) : new Pose(45.75,34.75,Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(123.25, 20.75,0) : new Pose(98.25, 34.75,Math.toRadians(0)));
//    public static Pose FIELD_OFFSET;
//    public static Pose CamOff;
    public Pose parkPose = SharedData.red ? new Pose(38.375, 32.25,0) :  new Pose(105.625,32.25,Math.toRadians(270));
    public Pose farLaunch = !SharedData.red ? new Pose(56.75,14.75,Math.toRadians(294)) : new Pose(87.25,14.75,Math.toRadians(246));
    public Pose closeLaunch  = !SharedData.red ? new Pose(56.75,80.75,Math.toRadians(315)) :  new Pose(87.25,80.75,Math.toRadians(225));
}
