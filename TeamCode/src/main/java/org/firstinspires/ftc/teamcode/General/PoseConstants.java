package org.firstinspires.ftc.teamcode.General;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;


@Configurable
public class PoseConstants {
    public Pose START_POSE =  SharedData.side == Side.BLUE ? (SharedData.startFar ? new Pose(56.75,9.36,Math.toRadians(270)) : new Pose(27.85, 125.85, Math.toRadians(190.75))) :  (SharedData.startFar ? new Pose(87.25,8.75,Math.toRadians(270)) : new Pose(118.55, 125.25, Math.toRadians(350)));
    public Pose LAUNCH_POSE = SharedData.side == Side.BLUE ? (SharedData.shootFar ? new Pose(56.75,14.75,Math.toRadians(290)) : new Pose(56.75,80.75,Math.toRadians(312))) :  (SharedData.shootFar ? new Pose(87.25,14.75,Math.toRadians(246)) : new Pose(87.25,80.75,Math.toRadians(225)));
    public Pose LAUNCH_POSE_ALT = SharedData.side == Side.BLUE ? (SharedData.shootFar ? new Pose(87.75 , 14.75 , Math.toRadians(312)) : new Pose(84 , 116 , Math.toRadians(345))) : (SharedData.shootFar ? new Pose(56.75 , 14.75 , Math.toRadians(225)) : new Pose(60 , 116 , Math.toRadians(195)));
    public Pose ALIGN1_POSE = SharedData.side == Side.BLUE ? (SharedData.shootFar ? new Pose(45.75,33.25,Math.toRadians(180)) : new Pose(45.75,81.25,Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(98.25,33.25,Math.toRadians(0)) : new Pose(102,83,Math.toRadians(0)));

    public Pose COMBINED_ALIGN1_POSE = SharedData.side == Side.BLUE ? (new Pose(45.75,33.25,Math.toRadians(180)) ):  new Pose(98.25,33.25,Math.toRadians(0));
    public Pose COMBINED_PICKUP1_POSE = SharedData.side == Side.BLUE ? (new Pose(17.75,33.25, Math.toRadians(180)) ):  new Pose(126.25,33.25, Math.toRadians(0));
    public Pose COMBINED_ALIGN1_CONTROL = SharedData.side == Side.BLUE ? (new Pose(68, 34.5)) : (new Pose(76,33.5));
    public Pose COMBINED_PICKUP1_CONTROL = SharedData.side == Side.BLUE ? (new Pose(70,34.5 )) : (new Pose(74, 34.5));
//    public Pose COMBINED_ENDPOSE = SharedData.side == Side.BLUE ? (new Pose(17.75, 50.25, Math.toRadians(180)) ) : (new Pose(126.25, 59.25, Math.toRadians(0)) );


    public Pose COMBINED_ALIGN2_POSE = SharedData.side == Side.BLUE ? (new Pose(15.75, 12, Math.toRadians(180)) ) : (new Pose(120, 12, Math.toRadians(0)) );
    public Pose COMBINED_ALIGN2_CONTROL = SharedData.side == Side.BLUE? (new Pose(70, 5)) : (new Pose (70, 30));

//    public Pose COMBINED_PICKUP2_POSE = SharedData.side == Side.BLUE? () : ();
//    public Pose COMBINED_PICKUP2_CONTROL = SharedData.side == Side.BLUE ? () : ();
//    public ArrayList<Pose> curve = new ArrayList<Pose>(Arrays.asList(new Pose()));
//    public BezierCurve test = new BezierCurve(curve);

    public  Pose PICKUP1_POSE = SharedData.side == Side.BLUE ? (SharedData.shootFar ? new Pose(17.75,33.25, Math.toRadians(180)) : new Pose(17.75,81.25, Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(126.25,33.25, Math.toRadians(0)) : new Pose(128,83, Math.toRadians(0)));
    public Pose ALIGN2_POSE = SharedData.side == Side.BLUE ? new Pose(45.75,59.25,Math.toRadians(180)) :  new Pose(102.25,59.25,Math.toRadians(0));
    public Pose PICKUP2_POSE = SharedData.side == Side.BLUE ? new Pose(17.75, 59.25, Math.toRadians(180)) :  new Pose(126.25, 59.25, Math.toRadians(0));
    public Pose END_POSE = SharedData.side == Side.BLUE ? (SharedData.shootFar ? new Pose(56.75,30.75,Math.toRadians(240)) : new Pose(45.75,69.75,Math.toRadians(180))) :  (SharedData.shootFar ? new Pose(87.25, 30.75,0) : new Pose(98.25, 69.75,Math.toRadians(0)));

//    public static Pose FIELD_OFFSET;
    public Pose parkPose = SharedData.side == Side.RED ? new Pose(38.375, 32.25,0) :  new Pose(105.625,32.25,Math.toRadians(270));
    public Pose gatePose = SharedData.side == Side.RED ? new Pose(126 , 78, Math.toRadians(0)) : new Pose(12, 58.5 , Math.toRadians(152));
    public Pose farAlt = SharedData.side == Side.BLUE ? new Pose(87.75 , 14.75 , Math.toRadians(301)) : new Pose(56.75 , 14.75 , Math.toRadians(234));
    public Pose closeAlt = SharedData.side == Side.BLUE ? new Pose(84 , 116 , Math.toRadians(345)) : new Pose(60 , 116 , Math.toRadians(192));
    public Pose farLaunch = SharedData.side == Side.BLUE ? new Pose(56.75,14.75,Math.toRadians(291)) : new Pose(87.25,14.75,Math.toRadians(246));
    public Pose closeLaunch  = SharedData.side == Side.BLUE ? new Pose(56.75,80.75,Math.toRadians(310)) :  new Pose(87.25,80.75,Math.toRadians(225));




    public Pose ALIGN3_POSE = SharedData.side == Side.RED ? new Pose(121,11,Math.toRadians(340)) : new Pose(27,11,Math.toRadians(200));
    //public Pose ALIGN3_POSE_CONTROL = SharedData.side == Side.RED ? new Pose(124, 6) : new Pose(28, 6);
    public Pose PICKUP3_POSE = SharedData.side == Side.RED ? new Pose(137,11,Math.toRadians(340)) : new Pose(11,11,Math.toRadians(200));
}
