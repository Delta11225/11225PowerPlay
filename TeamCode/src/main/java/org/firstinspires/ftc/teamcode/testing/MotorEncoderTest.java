package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Disabled
public class MotorEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor front_left = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor front_right = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor rear_right = hardwareMap.get(DcMotor.class, "rear_right");

        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        rear_left.setDirection(DcMotorSimple.Direction.FORWARD);
        rear_right.setDirection(DcMotorSimple.Direction.FORWARD);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Init complete!");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Starting in a second or two...");
        telemetry.update();
        sleep(1500);
        resetRuntime();

        while (getRuntime() <= 2) {
            // Quick stop if needed
            if (!gamepad1.atRest() || !gamepad2.atRest()) {
                break;
            }

            front_left.setPower(0.5);
            front_right.setPower(0.5);
            rear_left.setPower(0.5);
            rear_right.setPower(0.5);

            telemetry.addData("FL reported power", front_left.getPower());
            telemetry.addData("FR reported power", front_right.getPower());
            telemetry.addData("RL reported power", rear_left.getPower());
            telemetry.addData("RR reported power", rear_right.getPower());

            telemetry.addData("FL reported encoders", front_left.getCurrentPosition());
            telemetry.addData("FR reported encoders", front_right.getCurrentPosition());
            telemetry.addData("RL reported encoders", rear_left.getCurrentPosition());
            telemetry.addData("RR reported encoders", rear_right.getCurrentPosition());

            telemetry.update();
        }
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

        sleep(10000);
    }
}
