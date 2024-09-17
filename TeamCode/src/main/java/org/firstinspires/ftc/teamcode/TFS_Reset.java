package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "TFS_Reset", group = "FTC2023")
public class TFS_Reset extends LinearOpMode {
    private DcMotor leftArmMtr = null;
    private DcMotor rightArmMtr,ClawArm = null;
    private TouchSensor slideTouchR = null;
    private TouchSensor slideTouchL = null;
//testpull
    @Override
    public void runOpMode() throws InterruptedException {
        leftArmMtr = hardwareMap.get(DcMotorEx.class, "armL");
        rightArmMtr = hardwareMap.get(DcMotorEx.class, "armR");
        leftArmMtr.setDirection(DcMotor.Direction.FORWARD);
        leftArmMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArmMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMtr.setDirection(DcMotor.Direction.REVERSE);
        rightArmMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //        slideTouchR = hardwareMap.get(TouchSensor.class, "RightTouch");
//        slideTouchL = hardwareMap.get(TouchSensor.class, "LeftTouch");
//        rightArmMtr.setDirection(DcMotor.Direction.REVERSE);
//
//        while(!isStopRequested() &&!slideTouchL.isPressed()){
//            telemetry.addLine("Detect Left touch sensor for slide reset");
//            telemetry.update();
//            sleep(10);
//        }
//        rightArmMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightArmMtr.setPower(-0.3);
//        while(!isStopRequested() &&!slideTouchR.isPressed()){
//            telemetry.addLine("Detect Left touch sensor for slide reset");
//            telemetry.update();
//            sleep(10);
//        }
//        rightArmMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightArmMtr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            telemetry.addLine("Right Slide Reset Done");
        leftArmMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClawArm = hardwareMap.get(DcMotor.class, "arm");
        ClawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ClawArm.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

        }
    }
}