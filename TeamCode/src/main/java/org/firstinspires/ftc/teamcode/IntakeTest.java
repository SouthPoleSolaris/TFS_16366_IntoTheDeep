package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IntakeTest", group = "FTC2023")
public class IntakeTest extends LinearOpMode {


    private DcMotor dronemotor = null;
    private CRServo droneservo = null;

    private double speed;
    @Override
    public void runOpMode() {


        // CHANGE TO GAMEPAD 2 LATER & Commit / Push


        dronemotor = hardwareMap.get(DcMotor.class, "Intake");
        droneservo = hardwareMap.get(CRServo.class, "drone");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        double speed = 0.5;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            dronemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.a) {
                droneservo.setPower(-1);
            } else {
                droneservo.setPower(0);
            }
            if (gamepad2.y) {
                dronemotor.setPower(1);
            }

            if (gamepad2.x) {
                dronemotor.setPower(0);
            }

            if (gamepad2.b) {
                dronemotor.setPower(-speed);
            }

//            if (gamepad2.x) {
//                droneservo.setPower(0);
//                dronemotor.setPower(0);
//            }
            if (gamepad2.dpad_down){
                speed-=0.01;
            }
            else if (gamepad2.dpad_up){
                speed+=0.01;
            }
            else if (gamepad2.dpad_left) speed += 0.1;
            else if (gamepad2.dpad_right) speed -= 0.1;

            if (Math.abs(speed)>1) speed = 1;
            else if (Math.abs(speed)<0) speed = 0;


            telemetry.addData("currentspeed", speed);
            telemetry.update();
            sleep(150);
        }
    }
}
