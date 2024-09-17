package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.TeamElementDetection.TeamElementSubsystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TFS_Auton_2024", group="Auto")

public class  TFS_Auton_2024 extends LinearOpMode{

    public int element_zone = 1;
    public static double DISTANCE = 90; // in
    public boolean isLeft = false;
    public boolean isBlue = false;
    public static int blueVal = 1;
    private DcMotor ClawArm = null;
    private Servo ClawGrabL = null;
    private Servo ClawGrabR = null;
    private Servo ClawWristL = null;
    private Servo ClawWristR = null;
    private TeamElementSubsystem teamElementDetection=null;


    boolean togglePreview = true;
    ArmSystem armSystem;


    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        teamElementDetection = new TeamElementSubsystem(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();
        armSystem = new ArmSystem(this, true);
        ClawArm = hardwareMap.get(DcMotor.class, "arm");
        ClawWristL = hardwareMap.get(Servo.class, "ClawWristL");
        ClawWristR = hardwareMap.get(Servo.class, "ClawWristR");
        ClawGrabL = hardwareMap.get(Servo.class, "ClawGrabL");
        ClawGrabR = hardwareMap.get(Servo.class, "ClawGrabR");
        String curAlliance = "red";
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

                                    // -------------- AUTON TRAJECTORY ------------------------ //
                                         // -------------- ZONE 1 -------------- //
        // RED SIDE ** RIGHT** //  done, not tested
        Trajectory Zone1_1_RED_RIGHT = drive.trajectoryBuilder(startPose)
                .back(34)
                .build();
        Trajectory Zone1_2_RED_RIGHT = drive.trajectoryBuilder(Zone1_1_RED_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(90))))
                .back(10)
                .build();
        //claw open
        Trajectory Zone1_3_RED_RIGHT = drive.trajectoryBuilder(Zone1_2_RED_RIGHT.end())
                .forward(50)
                .build();

// Planned yellow deposit
//        Trajectory Zone1_4_RED_RIGHT = drive.trajectoryBuilder(Zone1_3_RED_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open




        // BLUE SIDE ** RIGHT** //done, not tested
        Trajectory Zone1_1_BLUE_RIGHT = drive.trajectoryBuilder(startPose)
                .back(30)
                .build();
        //turn right
        Trajectory Zone1_2_BLUE_RIGHT = drive.trajectoryBuilder(Zone1_1_BLUE_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
                .forward(15)
                .build();
        //claw open
        Trajectory Zone1_3_BLUE_RIGHT = drive.trajectoryBuilder(Zone1_2_BLUE_RIGHT.end())
                .forward(55)
                .build();
        // Planned yellow deposit
//        Trajectory Zone1_4_BLUE_RIGHT = drive.trajectoryBuilder(Zone1_3_BLUE_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open



        // RED SIDE ** LEFT ** //done, not tested
        Trajectory Zone1_1_RED_LEFT = drive.trajectoryBuilder(startPose)
                .back(30)
                .build();
        //turn left
        Trajectory Zone1_2_RED_LEFT = drive.trajectoryBuilder(Zone1_1_RED_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(90))))
                .back(15)
                .build();
        //claw open
        Trajectory Zone1_3_RED_LEFT = drive.trajectoryBuilder(Zone1_2_RED_LEFT.end())
                .forward(65)
                .build();
// Planned yellow deposit
//        Trajectory Zone1_4_RED_LEFT = drive.trajectoryBuilder(Zone1_3_RED_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open

        
        // BLUE SIDE ** LEFT ** // done, not tested
        Trajectory Zone1_1_BLUE_LEFT = drive.trajectoryBuilder(startPose)
                .strafeRight(45)
                .build();
        Trajectory Zone1_2_BLUE_LEFT = drive.trajectoryBuilder(startPose)
                .back(25)
                .build();
        Trajectory Zone1_3_BLUE_LEFT = drive.trajectoryBuilder(Zone1_3_RED_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
                .back(10)
                .build();
        //claw open
        Trajectory Zone1_4_BLUE_LEFT = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        // Planned yellow deposit
//        Trajectory Zone1_5_BLUE_LEFT = drive.trajectoryBuilder(Zone1_4_BLUE_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open


        // CHANGE INTO CORRECT SIDE & COLOR //
        Trajectory leftL = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(60)
                .build();
        Trajectory leftR = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(120)
                .build();


                                       
                                                    // -------------- ZONE 2 -------------- //
        // RED SIDE ** RIGHT** // done, not tested
        Trajectory Zone2_1_RED_RIGHT = drive.trajectoryBuilder(startPose)
                .back(40)
                .build();
        // claw open
        Trajectory Zone2_2_RED_RIGHT = drive.trajectoryBuilder(Zone2_1_RED_RIGHT.end())
                .forward(12)
                .build();
        Trajectory Zone2_3_RED_RIGHT = drive.trajectoryBuilder(Zone2_2_RED_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
                .back(40)
                .build();
// Planned yellow deposit
//        Trajectory Zone2_4_RED_RIGHT = drive.trajectoryBuilder(Zone2_3_RED_RIGHT.end())
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open


        // BLUE SIDE ** RIGHT** //done, not tested
        Trajectory Zone2_1_BLUE_RIGHT = drive.trajectoryBuilder(startPose)
                .back(30)
                .build();
        //claw open
        Trajectory Zone2_2_BLUE_RIGHT  = drive.trajectoryBuilder(Zone2_1_BLUE_RIGHT.end())
                .forward(29)
                .build();
        Trajectory Zone2_3_BLUE_RIGHT  = drive.trajectoryBuilder(Zone2_2_BLUE_RIGHT.end())
                .strafeRight(60)
                .build();
// Planned yellow deposit
//        Trajectory Zone2_4_BLUE_RIGHT = drive.trajectoryBuilder(Zone2_3_BLUE_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(90))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open



        // RED SIDE ** LEFT ** // done, not tested
        Trajectory Zone2_1_RED_LEFT = drive.trajectoryBuilder(startPose)
                .back(30)
                .build();
        //claw open
        Trajectory Zone2_2_RED_LEFT = drive.trajectoryBuilder(Zone2_1_RED_LEFT.end())
                .forward(29)
                .build();
        Trajectory Zone2_3_RED_LEFT = drive.trajectoryBuilder(Zone2_2_RED_LEFT.end())
                .strafeLeft(60)
                .build();
// Planned yellow deposit
//        Trajectory Zone2_4_RED_LEFT = drive.trajectoryBuilder(Zone2_3_RED_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open



        // BLUE SIDE ** LEFT ** // done, not tested
        Trajectory Zone2_1_BLUE_LEFT = drive.trajectoryBuilder(startPose)
                .back(40)
                .build();
        // claw open
        Trajectory Zone2_2_BLUE_LEFT = drive.trajectoryBuilder(Zone2_1_RED_RIGHT.end())
                .forward(12)
                .build();
        Trajectory Zone2_3_BLUE_LEFT = drive.trajectoryBuilder(Zone2_2_RED_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(90))))
                .back(60)
                .build();
        // Planned yellow deposit
//        Trajectory Zone2_4_BLUE_LEFT = drive.trajectoryBuilder(Zone2_3_BLUE_LEFT.end())
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open


        // -------------- ZONE 3 -------------- //
        // RED SIDE ** RIGHT ** // done, not tested
        Trajectory Zone3_1_RED_RIGHT = drive.trajectoryBuilder(startPose)
                .strafeLeft(27)
                .build();
        Trajectory Zone3_2_RED_RIGHT = drive.trajectoryBuilder(Zone3_1_RED_RIGHT.end())
                .back(20)
                .build();
        //claw open
        Trajectory Zone3_3_RED_RIGHT = drive.trajectoryBuilder(Zone3_2_RED_RIGHT.end())
                .forward(27)
                .build();
        Trajectory Zone3_4_RED_RIGHT = drive.trajectoryBuilder(Zone3_3_RED_RIGHT.end())
                .strafeLeft(30)
                .build();
        // Planned yellow deposit
//        Trajectory Zone3_5_RED_RIGHT = drive.trajectoryBuilder(Zone3_4_RED_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open

        
        // BLUE SIDE ** RIGHT ** // done, not tested
        Trajectory Zone3_1_BLUE_RIGHT = drive.trajectoryBuilder(startPose)
                .back(37)
                .build();
        //turn right
        Trajectory Zone3_2_BLUE_RIGHT = drive.trajectoryBuilder(Zone3_1_BLUE_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(-90))))
                .back(10)
                .build();
        //claw open
        Trajectory Zone3_3_BLUE_RIGHT = drive.trajectoryBuilder(Zone3_2_BLUE_RIGHT.end())
                .strafeRight(5)
                .build();
        Trajectory Zone3_4_BLUE_RIGHT = drive.trajectoryBuilder(Zone3_3_BLUE_RIGHT.end())
                .forward(60)
                .build();
// Planned yellow deposit
//        Trajectory Zone3_5_BLUE_RIGHT = drive.trajectoryBuilder(Zone3_4_BLUE_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open

        
        // RED SIDE ** LEFT ** // done, not tested
        Trajectory Zone3_1_RED_LEFT = drive.trajectoryBuilder(startPose)
                .back(34)
                .build();
        //turn left
        Trajectory Zone3_2_RED_LEFT = drive.trajectoryBuilder(Zone3_1_RED_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(90))))
                .forward(16)
                .build();
        //claw open
        Trajectory Zone3_3_RED_LEFT = drive.trajectoryBuilder(Zone3_2_RED_LEFT.end())
                .forward(55)
                .build();
        // Planned yellow deposit
//        Trajectory Zone3_4_RED_LEFT = drive.trajectoryBuilder(Zone3_4_LEFT_RIGHT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open

        // BLUE SIDE ** LEFT ** // done, not tested

        Trajectory Zone3_1_BLUE_LEFT = drive.trajectoryBuilder(startPose)
                .back(30)
                .build();
        Trajectory Zone3_2_BLUE_LEFT = drive.trajectoryBuilder(Zone3_1_BLUE_LEFT.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .back(15)
                .build();
        //claw open
        Trajectory Zone3_3_BLUE_LEFT = drive.trajectoryBuilder(Zone3_2_BLUE_LEFT.end())
                .forward(55)
                .build();
// Planned yellow deposit
//        Trajectory Zone3_4_BLUE_LEFT = drive.trajectoryBuilder(Zone3_3_BLUE_LEFT.end().plus(new Pose2d(0, 0,Math.toRadians(180))))
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open

        // CHANGE NAME FOR SIDE & COLOR //
        Trajectory foward = drive.trajectoryBuilder(startPose)
                .back(11)
                .build();

        while (!opModeIsActive() && !isStopRequested()){
            element_zone = teamElementDetection.elementDetection(telemetry);
            telemetry.addData("getMaxDistance", teamElementDetection.getMaxDistance());
            telemetry.addData("Element Zone 1 distance", teamElementDetection.getzone1distance());
            telemetry.addData("Element Zone 2 distance", teamElementDetection.getzone2distance());

            if (togglePreview && gamepad2.a){
                togglePreview = false;
                teamElementDetection.toggleAverageZone();
            }else if (!gamepad2.a){
                togglePreview = true;
            }


            if (gamepad1.x){
                curAlliance = "blue";
                isBlue = true;
                blueVal = -1;
            }else if (gamepad1.b){
                curAlliance = "red";
                isBlue = false;
            }
            if(gamepad1.y){
                if(!isBlue){
                    isLeft = true;
                }
                else {
                    isLeft = false;
                }
            }
            else if(gamepad1.a){
                if(!isBlue){
                    isLeft = false;
                }
                else {
                    isLeft = true;
                }
            }
            teamElementDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Current Side Selected is Left?: ", isLeft);
            telemetry.addData("Side?: ", isBlue);
            telemetry.addData("IsLeft (Gamepad1 Y = true, Gamepad1 A = false):", isLeft);
            telemetry.update();
        }

        telemetry.addData("Object", "Passed waitForStart");


        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        if(element_zone==1){
            if(isLeft) {
                if(isBlue){
                    drive.followTrajectory(Zone1_1_BLUE_LEFT);
                    drive.followTrajectory(Zone1_2_BLUE_LEFT);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(Zone1_3_BLUE_LEFT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone1_4_BLUE_LEFT);
//                    drive.followTrajectory(Zone1_5_BLUE_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else{
                   drive.followTrajectory(Zone1_1_RED_LEFT);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(Zone1_2_RED_LEFT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone1_3_RED_LEFT);
//                    drive.followTrajectory(Zone1_4_RED_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }


            }
            else{
                if(isBlue){
                    drive.followTrajectory(Zone1_1_BLUE_RIGHT);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(Zone1_2_BLUE_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone1_3_BLUE_RIGHT);
//                    drive.followTrajectory(Zone1_4_BLUE_RIGHT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else {
                    drive.followTrajectory(Zone1_1_RED_RIGHT);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(Zone1_2_RED_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone1_3_RED_RIGHT);
//                    drive.followTrajectory(Zone1_4_RED_RIGHT);
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }

            }

        }
        else if(element_zone==2){
            if(isLeft) {
                if(isBlue){
                    drive.followTrajectory(Zone2_1_BLUE_LEFT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone2_2_BLUE_LEFT);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(Zone2_3_BLUE_LEFT);
//                    drive.followTrajectory(Zone2_4_BLUE_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else{
                    drive.followTrajectory(Zone2_1_RED_LEFT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone2_2_RED_LEFT);
                    drive.followTrajectory(Zone2_3_RED_LEFT);
//                    drive.followTrajectory(Zone2_4_RED_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
            }
            else{
                if(isBlue){
                    drive.followTrajectory(Zone2_1_BLUE_RIGHT);
                    drive.followTrajectory(Zone2_2_BLUE_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone2_3_BLUE_RIGHT);
//                    drive.followTrajectory(Zone2_4_BLUE_RIGHT);
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else {
                    drive.followTrajectory(Zone2_1_RED_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone2_2_RED_RIGHT);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(Zone2_3_RED_RIGHT);
//                    drive.followTrajectory(Zone2_4_RED_RIGHT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
            }

        }
        else if(element_zone==3){
            if(isLeft){
                if(isBlue){
                    drive.followTrajectory(Zone3_1_BLUE_LEFT);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(Zone3_2_BLUE_LEFT);
                    ClawGrabL.setPosition(0.5);
                    drive.followTrajectory(Zone3_3_BLUE_LEFT);
//                    drive.followTrajectory(Zone3_4_BLUE_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else{
                    drive.followTrajectory(Zone3_1_RED_LEFT);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(Zone3_2_RED_LEFT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone3_3_RED_LEFT);
//                    drive.followTrajectory(Zone3_4_RED_LEFT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }


            }
            else{
                if(isBlue){
                    drive.followTrajectory(Zone3_1_BLUE_RIGHT);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(Zone3_2_BLUE_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone3_3_BLUE_RIGHT);
                    drive.followTrajectory(Zone3_4_BLUE_RIGHT);
//                    drive.followTrajectory(Zone3_5_BLUE_RIGHT);
//        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }
                else {
                    drive.followTrajectory(Zone3_1_RED_RIGHT);
                    drive.followTrajectory(Zone3_2_RED_RIGHT);
                    ClawGrabL.setPosition(0.5); //open
                    drive.followTrajectory(Zone3_3_RED_RIGHT);
                    drive.followTrajectory(Zone3_4_RED_RIGHT);
//                    drive.followTrajectory(Zone3_5_RED_RIGHT);
                    //        ClawArm.setTargetPosition(200); //deposit
//        ClawWristL.setPosition(0.9); //deposit
//        ClawWristR.setPosition(0.1); //deposit
//        ClawGrabR.setPosition(0.4); //open
                }

            }

        }
        PoseStorage.currentPose = drive.getPoseEstimate();
        while (!isStopRequested() && opModeIsActive()) ;


    }

}
