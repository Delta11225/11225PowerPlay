package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Drive Forward")
public class DriveForward extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");

        telemetry.addData("Mode", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "Running");
        telemetry.update();

        if (gamepad1.right_bumper) {
            leftMotor.setPower(0.25);
        } else if (gamepad1.left_bumper){
            leftMotor.setPower(-0.25);
        } else {
            leftMotor.setPower(0);
        }


        // rightMotor.setPower(0.25);

//        sleep(2000);
//
//        leftMotor.setPower(-0.25);
//        // rightMotor.setPower(-0.25);
//
//        sleep(2000);
//
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
    }
}
