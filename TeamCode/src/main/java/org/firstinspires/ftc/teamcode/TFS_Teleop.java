package org.firstinspires.ftc.teamcode;

import android.graphics.SweepGradient;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "TFS_TeleOp", group = "FTC2023")
public class TFS_Teleop extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    int armMin = 0;
    int armMax = 5000;
    double targetAngle = Math.toRadians(90 * TFS_Auton_2024.blueVal);
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
    private double dronespeed = 0.7;
    boolean droneMotorBool;
    boolean ClawGrabBool;
    public int armPos;
    boolean climbBool = false;
    IMU imu;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private boolean autoThreadFlag = false;
    private boolean traverseMode = false;
    private double clawRClose=0.46;
    private double clawROpen=0.65;
    private double clawLClose=0.54;
    private double clawLOpen=0.35;


    private int bButton_DelayCnt = 0;

    @Override
    public void runOpMode() throws InterruptedException {
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


//        ClawWristL.setPosition(0); //deposit
//        ClawWristR.setPosition(1); //deposit
//        ClawGrabL.setPosition(0.54); //close
//        ClawGrabR.setPosition(0.11); //close

        // arm max = -2300 - outtake position
        // arm min = -100 - intake position
        ClawArm.setTargetPosition(62);
        ClawArm.setPower(0.7);
        ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(50);
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //Pose2d poseEstimate = drive.getPoseEstimate();
            armPos = ClawArm.getCurrentPosition();

            // Update everything. Odometry. Etc.
            // drive.update();
            double max;

            double speedFactor;
            if (gamepad1.right_bumper) {
                speedFactor = 0.9;
            } else {
                speedFactor = 0.5;
            }

            // now the orientation of robot is changed
            double leftStickXPos = gamepad1.left_stick_x * speedFactor;
            double leftStickYPos = gamepad1.left_stick_y * speedFactor;
            double rightStickXPos = -gamepad1.right_stick_x * speedFactor;

            double denominator = Math.max(Math.abs(leftStickYPos) + Math.abs(leftStickXPos) + Math.abs(rightStickXPos), 1);

            double lfPower = (leftStickYPos - leftStickXPos - rightStickXPos) / denominator;
            double lrPower = (leftStickYPos + leftStickXPos - rightStickXPos) / denominator;
            double rfPower = (leftStickYPos + leftStickXPos + rightStickXPos) / denominator;
            double rrPower = (leftStickYPos - leftStickXPos + rightStickXPos) / denominator;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lrPower));
            max = Math.max(max, Math.abs(rrPower));

            if (max > 1.0) {
                lfPower /= max;
                rfPower /= max;
                lrPower /= max;
                rrPower /= max;
            }

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftRear.setPower(lrPower);
            rightRear.setPower(rrPower);

            armSystem.armTeleOp(gamepad2);
            //climb motor


            if (!armSystem.armflag && gamepad2.y && !autoThreadFlag) {
                autoThreadFlag = true;
                armSystem.armflag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {

                        ClawGrabL.setPosition(0.54); //close
                        ClawGrabR.setPosition(0.46);
                        sleep(600);
                        ClawArm.setTargetPosition(2200);
                        ClawArm.setPower(0.7);
                        ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sleep(2000);
                        ClawWristL.setPosition(0); //deposit
                        ClawWristR.setPosition(1); //deposit
                        sleep(100);
                        armSystem.sliderCtrl("both", 2000, 1);
                        sleep(600);
                        armSystem.armflag = false;
                        autoThreadFlag = false;
                    }
                }).start();
            }

            if (gamepad2.a && !armSystem.armflag && !autoThreadFlag) {
                autoThreadFlag = true;
                armSystem.armflag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {

                        ClawGrabL.setPosition(0.54); //close
                        ClawGrabR.setPosition(0.46);
                        sleep(100);
                        ClawWristL.setPosition(0.28); //floor
                        ClawWristR.setPosition(0.72); //floor
                        sleep(800);
                        ClawArm.setTargetPosition(62);
                        ClawArm.setPower(0.7);
                        ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sleep(50);
                        armSystem.sliderCtrl("both", 0, 1);
                        sleep(1500);
                        ClawGrabL.setPosition(0.35); //open
                        ClawGrabR.setPosition(0.65); //open
                        armSystem.armflag = false;
                        autoThreadFlag = false;
                    }
                }).start();
            }

            if(gamepad2.right_bumper & !autoThreadFlag){
                autoThreadFlag = true;
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        dronemotor.setPower(dronespeed);
                        sleep(1300);
                        droneservo.setPower(-1);
                        sleep(3000);
                        droneservo.setPower(0);
                        dronemotor.setPower(0);
                    }
                }).start();
            }

            if (gamepad2.b && bButton_DelayCnt == 0 && !ClawGrabBool) {
                bButton_DelayCnt = 10;
                ClawGrabL.setPosition(0.54); //close
                ClawGrabR.setPosition(0.46);
                ClawGrabBool = true;
                // sleep(100);
            } else if (gamepad2.b && bButton_DelayCnt == 0 && ClawGrabBool) {
                bButton_DelayCnt = 10;
                ClawGrabL.setPosition(0.35); //open
                ClawGrabR.setPosition(0.65); //open

                ClawGrabBool = false;
                //sleep(100);
            } else {
                bButton_DelayCnt--;
                bButton_DelayCnt = bButton_DelayCnt < 0 ? 0 : bButton_DelayCnt;
            }

            if (gamepad2.dpad_up){
                ClawWristL.setPosition(0.28); //floor
                ClawWristR.setPosition(0.72); //floor
            }

            if (gamepad2.right_bumper) {
                ClawGrabL.setPosition(0.54); //close
                ClawGrabR.setPosition(0.46);
                ClawArm.setTargetPosition(400);
                ClawArm.setPower(0.7);
                ClawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.dpad_down){
                ClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(gamepad2.dpad_left){
                ClawGrabR.setPosition(0.65); //open
            }

            telemetry.addData("Current claw position", armPos);
            telemetry.addData("claw grab data", ClawGrabBool);
            telemetry.addData("slideL", armSystem.leftArmMtr.getCurrentPosition());
            telemetry.addData("slideR", armSystem.rightArmMtr.getCurrentPosition());
            telemetry.addData("state", armSystem.armflag);
            if (autoThreadFlag) {
                telemetry.addLine("thread start");
            } else {
                telemetry.addLine("thread end");
            }
            telemetry.update();
            sleep(33);
        }

    }
}