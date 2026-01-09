package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;


import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.PoseConstants;


import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.ColorSensed;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SharedData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "TeleOp")
public class TeleOpAllInOne extends LinearOpMode{
    private Robot hornet = new Robot();
    private Follower f;
    private PoseConstants poses = new PoseConstants();
    boolean robotCentric;
    boolean autoMode = true;
    boolean xButton;
    boolean dpadLeft2 = false;
    double speedMultiplier = 1;
    boolean dpadLeft;
    boolean launching;
    boolean automated;
    private Timer launchTimer;
    private int inMotif = 0;
//    private Limelight3A limelight;
//    private LLResult result;
    private TouchSensor touchSensor = null;
//    private static AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//    private AprilTagDetection currentDetection;
    int crashed = 0;
    String crashName = "";

    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    boolean slowMode;

    public void runOpMode() throws InterruptedException
    {
        hornet.initialize(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose == null ? poses.START_POSE : SharedData.toTeleopPose);
        f.update();
        launchTimer = new Timer();
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        hornet.resetHammer();


        waitForStart();
//        hornet.setStoragePos(hornet.getSlotGoal(), true);
        f.startTeleopDrive(true);

        while(opModeIsActive()) {
            int ID = 0;
//            if (result != null && result.isValid()){
//
//            }
            try {
                f.update();
            } catch (Exception e){
                crashName = e.getMessage();
                crashed++;
                telemetry.addData("crashed" , e.getMessage() );

            }
            tele();

            //
            if(!automated) {

                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        robotCentric,
                        (SharedData.red || robotCentric) ? 0 : Math.toRadians(180)
                );
            }

            if (!automated){
                //go to close launch main
                if (gamepad1.a){
                    f.followPath(goToClose(false));
                    automated = true;
                }
                //go to close launch alt
                else if (gamepad2.a  && !gamepad2.right_bumper && !gamepad2.left_bumper){
                    f.followPath(goToClose(true));
                    automated = true;
                }
                //go to park
                else if (gamepad1.x){
                    f.followPath(goToPark());
                    automated = true;
                }
                //go to far launch main
                else if (gamepad1.b){
                    f.followPath(goToFar(false));
                    automated = true;
                }
                //go to far launch alt
                else if (gamepad2.b && !gamepad2.right_bumper && !gamepad2.left_bumper ){
                    f.followPath(goToFar(true));
                    automated = true;
                }
                //go to gate
                else if ((gamepad1.y ||(gamepad2.y && !gamepad2.right_bumper && !gamepad2.left_bumper))) {
                    f.followPath(goToGate());
                    automated = true;
                }

            }
            //exits automated
            else if (!(gamepad1.a || (gamepad2.a && !gamepad2.right_bumper && !gamepad2.left_bumper)) && !(gamepad1.y || (gamepad2.y && !gamepad2.right_bumper && !gamepad2.left_bumper)) && !(gamepad1.b || (gamepad2.b && !gamepad2.right_bumper && !gamepad2.left_bumper)) && !gamepad1.x) {
                f.startTeleopDrive(true);
                automated = false;
            }
//            //Launch close main
//            if (gamepad1.a && !automated) {
//                f.followPath(goToClose(false));
//                automated = true;
//            }
//            //launch close alt
//            else if (gamepad2.a && !automated){
//                f.followPath(goToClose(true));
//                automated = true;
//            }
//
//            //Auto Pathing to Park
//            else if (gamepad1.x && !automated) {
//                f.followPath(goToPark());
//                automated = true;
//            }
//            //launch far main
//            else if (gamepad1.b && !automated) {
//                f.followPath(goToFar(false));
//                automated = true;
//            }
//            else if (gamepad2.b && !automated){
//                f.followPath(goToFar(true));
//                automated = true;
//            }
//
//            else if ((gamepad1.y ||(gamepad2.y && !gamepad2.right_bumper && !gamepad2.left_bumper)) && !automated){
//                f.followPath(goToGate());
//                automated = true;
//            }
//
//            //exits automated pathing
//            else if (!gamepad1.a && !gamepad1.x && !gamepad1.b && !(gamepad1.y || (gamepad2.y && !gamepad2.right_bumper && !gamepad2.left_bumper)) && automated) {
//                f.startTeleopDrive(true);
//                automated = false;
//            }


            if (gamepad1.dpad_right && gamepad1.right_trigger > .5){
                f.setPose(new Pose(f.getPose().getX() , 8.75 , f.getHeading()));
            }



//            if(slowMode != gamepad1.y && gamepad1.y)
//                speedMultiplier = speedMultiplier == 1 ? .2 : 1;
//            slowMode = gamepad1.y;

            if(gamepad1.left_trigger >= .2)
                speedMultiplier = .2;
            else
                speedMultiplier = 1;


            if(xButton != gamepad2.x && gamepad2.x)
                autoMode = !autoMode;
            xButton = gamepad2.x;
            if(SharedData.isFull()&&!launching && hornet.atSortTarget())
                hornet.startIntake(false,.2);
            else if(gamepad1.right_bumper)
                hornet.startIntake(true,1);
            else if(gamepad1.left_bumper)
                hornet.startIntake(false,1);
            else
                hornet.stopIntake();


            if (autoMode) {autoMode();} else {manualMode();}

            hornet.updateLED();

            if(dpadLeft != gamepad1.dpad_left && gamepad1.dpad_left)
                robotCentric = !robotCentric;
            dpadLeft = gamepad1.dpad_left;




        }


    }




    public void autoMode() {

        if((gamepad2.right_trigger > .2 && gamepad2.left_trigger > .2) || gamepad1.dpad_down)
        {
            launching = false;
            hornet.resetLaunch();
        }


        if(!launching)
            hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , !SharedData.isFull());

        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.clearSlot(0);
            else if(gamepad2.b)
                SharedData.clearSlot(1);
            else if(gamepad2.y)
                SharedData.clearSlot(2);
        }
        else if(gamepad2.right_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.GREEN;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.GREEN;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.GREEN;
        }
        else if(gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.PURPLE;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.PURPLE;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.PURPLE;
        }


        if(hornet.buttonPressed() && hornet.atSortTargetLenient() && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            SharedData.storage[hornet.getSlotGoal()] = hornet.detectColor();
        }


        if(gamepad2.dpad_up && SharedData.getGreenIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getGreenIndex(), false);
            launching = true;
        }
        if(gamepad2.dpad_down && SharedData.getPurpleIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getPurpleIndex(), false);
            launching = true;
        }
        if(gamepad2.dpad_right && !SharedData.isEmpty() && !launching)
        {
            hornet.setStoragePos(inMotif == SharedData.greenIndex ? (SharedData.getGreenIndex() == -1 ? SharedData.getPurpleIndex() : SharedData.getGreenIndex()) : (SharedData.getPurpleIndex() == -1 ? SharedData.getGreenIndex() : SharedData.getPurpleIndex()), false);
            launching = true;
        }
        if(dpadLeft2 != gamepad2.dpad_left && gamepad2.dpad_left)
            inMotif  = inMotif == 2 ? 0 : inMotif + 1;
        dpadLeft2 = gamepad2.dpad_left;


        //If launching -> speed up launchMotors
        if(launching){
			if(SharedData.red ? (f.getPose().getX() < 72) : (f.getPose().getX() > 72))
            	hornet.startLaunchMotorsAlt(f.getPose().getY() < 48);
			else
				hornet.startLaunchMotors(f.getPose().getY() < 48);
            hornet.startIntake(true, .1);
        } else{hornet.stopLaunchMotors();}

        //if ready to launch -> then launch
        if(launching && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.hammerAtLaunch() && !hornet.isLaunched()){
            hornet.launch();
            launchTimer.resetTimer();
        }

        //if flap has had time to move...
        //and flap is at launch position -> move flap back and clear storage slot
        //and flap is at not launch position and it says its launching -> say its not launching
        if(launchTimer.getElapsedTimeSeconds() >= .25){
            if(hornet.hammerAtLaunch() && launching){
                launchTimer.resetTimer();
                hornet.resetHammer();
                SharedData.clearSlot(hornet.getSlotGoal());
            }
            else if(launching && hornet.isLaunched()){
                launching = false;
                inMotif  = inMotif == 2 ? 0 : inMotif + 1;
                hornet.resetLaunch();
            }
        }


    }
    public void manualMode() {
        //sorter
        if(gamepad2.a)
            hornet.setStoragePos(0, !gamepad2.dpad_left);
        else if(gamepad2.b)
            hornet.setStoragePos(1, !gamepad2.dpad_left);
        else if(gamepad2.y)
            hornet.setStoragePos(2, !gamepad2.dpad_left);

        //launch
        if(gamepad2.right_trigger > .2)
            hornet.startLaunchMotors(true);
        else if(gamepad2.left_trigger > .2)
            hornet.startLaunchMotors(false);
        else
            hornet.stopLaunchMotors();

        //hammer
        if (gamepad2.right_bumper){
            hornet.launch();
        }
        else hornet.resetHammer();

    }
//    public void createPaths() {
//
//        toLaunchClose = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), poses.closeLaunch))
//                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.closeLaunch.getHeading())
//                .build();
//
//        toLaunchFar = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), poses.farLaunch))
//                .setConstantHeadingInterpolation(poses.farLaunch.getHeading())
//                .build();
//
//        toPark = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), poses.parkPose))
//                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
//                .build();
//    }


    public PathChain goToClose(boolean alt){
        if (alt){ return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.closeAlt))
                .setConstantHeadingInterpolation(poses.closeAlt.getHeading())
                .build();
            }
        else return f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.closeLaunch))
                .setConstantHeadingInterpolation(poses.closeLaunch.getHeading())
                .build();
    }
    public PathChain goToFar(boolean alt){
        if (alt){ return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.farAlt))
                .setConstantHeadingInterpolation(poses.farAlt.getHeading())
                .build();
        }
        else return f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.farLaunch))
                .setLinearHeadingInterpolation(f.getHeading() , poses.farLaunch.getHeading())
                .build();
    }


    public PathChain goToPark()
    {
        PathChain toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.parkPose))
                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
                .build();
        return toPark;
    }
//    public PathChain goToClose()
//    {
//        PathChain toLaunchClose = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), poses.closeLaunch))
//                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.closeLaunch.getHeading())
//                .build();
//        return toLaunchClose;
//    }

    public PathChain goToGate(){
        return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.gatePose))
                .setConstantHeadingInterpolation(poses.gatePose.getHeading())
                .build();

    }


//    public void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        builder.setCamera((CameraName) hardwareMap.get(Limelight3A.class, "limelight"));
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//    }
//    public static int figureID(){
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//        for (AprilTagDetection detection : detections){
//            if(detection.id == 21||detection.id == 22||detection.id == 23)
//                return detection.id;
//        }
//        return -1;
//    }


    public void tele() {
//        telemetry.addData("crashed" , crashed);
//        telemetry.addData("latest crash name" , crashName);
//            telemetry.addData("FOLLOWER X",f.getPose().getX());
//            telemetry.addData("FOLLOWER Y",f.getPose().getY());
//            telemetry.addData("FOLLOWER Heading",f.getPose().getHeading());
//        telemetry.addData("Pattern", SharedData.greenIndex);
//        telemetry.addLine(String.format("Storage: %s, %s, %s", SharedData.storage[0], SharedData.storage[1], SharedData.storage[2] ));
        telemetry.addData("auto Mode", autoMode);
//        telemetry.addData("Side", SharedData.red ? "Red" : "Blue");
//        telemetry.addData("Robot Centric" , robotCentric);
//        telemetry.addData("Slow Mode", slowMode);
//        telemetry.addLine(String.format("LeftVel: %f\nRightVel: %f",hornet.leftLaunch.getVelocity(), hornet.rightLaunch.getVelocity() ));
//        telemetry.addData("targetVelocity" ,hornet.getLaunchTargetVelocity());
//        telemetry.addData("atTarget" , hornet.atTargetVelocity());
//        telemetry.addData("launchTimer", launchTimer.getElapsedTimeSeconds());
//        telemetry.addData("button", hornet.buttonPressed());
//        telemetry.addData("at sort", hornet.atSortTarget());
//        telemetry.addData("goal clear", SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR);
//        telemetry.addData("current ID" , figureID());
//        telemetry.addData("fx" , f.getPose().getX());
//        telemetry.addData("fy" , f.getPose().getY());
//        telemetry.addData("fh" , f.getHeading());
//        telemetry.addData("automated" , automated);
        telemetry.addData("In Motif", inMotif);

//        telemetry.addData("\nColor Values\n", hornet.returnColor());


        telemetry.update();
    }


}
