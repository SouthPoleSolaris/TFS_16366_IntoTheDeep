package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TFS_Auton_World", group="Auto")

public class  TFS_Auton_World extends LinearOpMode{

    ArmSystem armSystem;
    private DcMotor dronemotor = null;
    private CRServo droneservo = null;
    private DcMotor ClawArm = null;

    private Servo ClawGrabL = null;
    private Servo ClawGrabR = null;
    private Servo ClawWristL = null;
    private Servo ClawWristR = null;
    private double speedfactor = 1.0;
    private double imuAngle = 0.0;
    private double dronespeed = 0.55;
    boolean droneMotorBool;
    boolean ClawGrabBool;
    public int armPos;
    IMU imu;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public boolean isLeft = false;
    public boolean isBlue = false;
    public static int blueVal = 1;

    private String curAlliance="Red";

    boolean togglePreview = true;


    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();


        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //Need to put armRight and armLeft in hardware map, for armsystem code
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive.setPoseEstimate(PoseStorage.currentPose);
        //DcMotor armLeftMotor = hardwareMap.get(DcMotor.class, "armLeft");
        armSystem = new ArmSystem(this, false);
        dronemotor = hardwareMap.get(DcMotor.class, "droneM");
        droneservo = hardwareMap.get(CRServo.class, "droneS");
        ClawArm = hardwareMap.get(DcMotor.class, "arm");
        ClawWristL = hardwareMap.get(Servo.class, "WristL");
        ClawWristR = hardwareMap.get(Servo.class, "WristR");
        ClawGrabL = hardwareMap.get(Servo.class, "ClawL");
        ClawGrabR = hardwareMap.get(Servo.class, "ClawR");
        dronemotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dronemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClawArm.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (!opModeIsActive() && !isStopRequested()){
             //deposit
            ClawGrabL.setPosition(0.54); //close
            ClawGrabR.setPosition(0.46); //close
            sleep(100);
            ClawArm.setTargetPosition(280);
            ClawArm.setPower(0.7);
            ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(200);
            ClawWristL.setPosition(0.05); //deposit
            ClawWristR.setPosition(0.95);
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
        if(isLeft){
            if(!isBlue){//red left
                setPower(0.5,-0.5,-0.5,0.5);
                sleep(300);
                setPower(0,0,0,0);
                sleep(7000);
                setPower(-0.65,-0.65,-0.69,-0.69);
                sleep(1250);
                setPower(0,0,0,0);
                sleep(1000);

//                setPower(0.6,-0.6,-0.62,0.62);
//                sleep(1910);
//                setPower(0,0,0,0);
//                sleep(3000);
//                setPower(-0.66,-0.66,-0.65,-0.65);
//                sleep(1600);
//                setPower(0,0,0,0);
//                sleep(1000);
//                setPower(-0.6,0.6,0.6,-0.6);
//                sleep(150);
//                setPower(0,0,0,0);

            }
            else{//blue left

                setPower(-0.5,-0.5,-0.5,-0.5);
                sleep(1000);
                setPower(0,0,0,0);
                sleep(50);

            }
        }
        else{
            if(!isBlue){//red right
                setPower(-0.5,-0.5,-0.5,-0.5);
                sleep(1000);
                setPower(0,0,0,0);
                sleep(50);

            }
            else{//blue right
                setPower(-0.5,0.5,0.5,-0.5);
                sleep(250);
                setPower(0,0,0,0);
                sleep(4000);
                setPower(-0.65,-0.65,-0.65,-0.65);
                sleep(1150);
                setPower(0,0,0,0);
                sleep(1000);

//                setPower(-0.5,0.5,0.5,-0.5);
//                sleep(250);
//                setPower(0,0,0,0);
//                sleep(30);
//                setPower(-0.65,-0.65,-0.65,-0.65);
//                sleep(2000);
//                setPower(0,0,0,0);
//                sleep(1000);
            }
        }
        ClawWristL.setPosition(0.28); //floor
        ClawWristR.setPosition(0.72); //floor
        sleep(600);
        ClawArm.setTargetPosition(62);
        ClawArm.setPower(0.7);
        ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        ClawGrabL.setPosition(0.35); //open
        ClawGrabR.setPosition(0.65); //open

        telemetry.addData("Current claw position", armPos);
        telemetry.update();
        while (!isStopRequested() && opModeIsActive()) ;

    }
    public void setPower(double lf, double lr, double rf, double rr){
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);

    }



}
