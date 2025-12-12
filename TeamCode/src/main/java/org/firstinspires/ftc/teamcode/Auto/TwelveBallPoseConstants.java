package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SharedData;

@Configurable
public class TwelveBallPoseConstants {
//    public Pose END_POSE = !SharedData.red ? (SharedData.shootFar ? new Pose(56.75,30.75,Math.toRadians(240)) : new Pose(45.75,69.75,Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(87.25, 30.75,0) : new Pose(98.25, 69.75,Math.toRadians(0)));
//
//    public Pose parkPose = SharedData.red ? new Pose(38.375, 32.25,0) :  new Pose(105.625,32.25,Math.toRadians(270));
//    public Pose gatePose = SharedData.red ? new Pose(126 , 78, Math.toRadians(0)) : new Pose(21, 78 , Math.toRadians(180));
//    public Pose farAlt = !SharedData.red ? new Pose(87.75 , 14.75 , Math.toRadians(301)) : new Pose(56.75 , 14.75 , Math.toRadians(239));
//    public Pose closeAlt = !SharedData.red ? new Pose(84 , 116 , Math.toRadians(345)) : new Pose(60 , 116 , Math.toRadians(195));
//    public Pose farLaunch = !SharedData.red ? new Pose(56.75,14.75,Math.toRadians(294)) : new Pose(87.25,14.75,Math.toRadians(246));
//    public Pose closeLaunch  = !SharedData.red ? new Pose(56.75,80.75,Math.toRadians(315)) :  new Pose(87.25,80.75,Math.toRadians(225));

    public Pose START_POSE =  !SharedData.red ? (SharedData.startFar ? new Pose(56.75,9.36,Math.toRadians(270)) : new Pose(27.85, 125.85, Math.toRadians(190.75))) :  (SharedData.startFar ? new Pose(87.25,8.75,Math.toRadians(270)) : new Pose(118.55, 125.25, Math.toRadians(350)));

    public Pose LAUNCH_POSE_FAR = !SharedData.red ? new Pose(56.75,14.75,Math.toRadians(292)) : new Pose(87.25,14.75,Math.toRadians(246));
    public Pose LAUNCH_POSE_CLOSE = !SharedData.red ? new Pose(56.75,80.75,Math.toRadians(312)) : new Pose(87.25,80.75,Math.toRadians(225));

    public Pose ALIGN1_POSE = !SharedData.red ? new Pose(45.75,59.25,Math.toRadians(180)) :  new Pose(102.25,59.25,Math.toRadians(0));
    public Pose PICKUP1_POSE = !SharedData.red ? new Pose(16, 62.25, Math.toRadians(180)) :  new Pose(137, 62.25, Math.toRadians(0));

    public Pose ALIGN2_POSE = !SharedData.red ? new Pose(45.75,33.25,Math.toRadians(180)) :  new Pose(102,33.25,Math.toRadians(0));
    public Pose PICKUP2_POSE = !SharedData.red ? new Pose(17.75,33.25, Math.toRadians(180)) : new Pose(126.25,33.25, Math.toRadians(0));

    public Pose ALIGN3_POSE = !SharedData.red ? new Pose(45.75,81.25,Math.toRadians(180)) : new Pose(102,83,Math.toRadians(0));
    public Pose PICKUP3_POSE = !SharedData.red ? new Pose(17.75,81.25, Math.toRadians(180)) : new Pose(128,83, Math.toRadians(0));

    public Pose END_POSE_FAR = !SharedData.red ? new Pose(56.75,30.75,Math.toRadians(240)) : new Pose(87.25, 30.75,0);
    public Pose END_POSE_CLOSE = !SharedData.red ? new Pose(45.75,69.75,Math.toRadians(180)) : new Pose(98.25, 69.75,Math.toRadians(0));
}
