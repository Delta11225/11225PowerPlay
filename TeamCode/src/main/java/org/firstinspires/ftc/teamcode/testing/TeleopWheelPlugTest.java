package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@TeleOp
public class TeleopWheelPlugTest extends LinearOpMode {

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;

    @Override
    public void runOpMode() {

        rearLeft = hardwareMap.dcMotor.get("rear_left");
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        rearRight = hardwareMap.dcMotor.get("rear_right");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        while (opModeIsActive()){

            if (gamepad1.a){
                frontLeft.setPower(0.5);
                sleep (1000);
                frontLeft.setPower(0);
                telemetry.addLine("front left");
                telemetry.addData("encoder:",frontLeft.getCurrentPosition());
                telemetry.update();
            }
            if (gamepad1.x){
                frontRight.setPower(0.5);
                sleep (1000);
                frontRight.setPower(0);
                telemetry.addData("encoder:",frontRight.getCurrentPosition());
                telemetry.addLine("front right");
                telemetry.update();
            }
            if (gamepad1.y){
                rearRight.setPower(-gamepad1.left_stick_y);
                telemetry.addLine("rear right");
            }
            if (gamepad1.b){
               rearLeft.setPower(-gamepad1.left_stick_y);
                telemetry.addLine("rear left");
            }

        }
    }
}
