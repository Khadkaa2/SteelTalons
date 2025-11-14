package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.SharedData;

@Configurable
public class PoseConstantsClose {

    public Pose LAUNCH_POSE = !SharedData.red ? new Pose(56.75,14.75,Math.toRadians(294)) :  new Pose(87.25,86.75,Math.toRadians(225));
    public Pose ALIGN1_POSE = !SharedData.red ? new Pose(45.75,34.75,Math.toRadians(180)) :  new Pose(98.25,82.75,Math.toRadians(0));
    public  Pose PICKUP1_POSE = !SharedData.red ? new Pose(17.75,34.75, Math.toRadians(180)) :  new Pose(126.25,82.75, Math.toRadians(0));
    public Pose ALIGN2_POSE = !SharedData.red ? new Pose(45.75,58.75,Math.toRadians(180)) :  new Pose(98.25,58.75,Math.toRadians(0));
    public Pose PICKUP2_POSE = !SharedData.red ? new Pose(17.75, 58.75, Math.toRadians(180)) :  new Pose(126.25, 58.75, Math.toRadians(0));
    public Pose END_POSE = !SharedData.red ? new Pose(20.75,20.75,Math.toRadians(240)) :  new Pose(98.25, 34.75,Math.toRadians(0));



}
