package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
//@Disabled
public class AutoBigClaw extends OpMode {

    DcMotor linearSlide;
    Servo rightClaw;
    Servo leftClaw;
    int holdPosition;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");

        sensorColor = hardwareMap.get(ColorSensor.class, "colordistance_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"colordistance_sensor");

        telemetry.addData("status", "Initialized");
        telemetry.update();

    }
    @Override
    public void loop() {

        if (sensorColor.blue() > sensorColor.red() && sensorDistance.getDistance(DistanceUnit.CM) <3 && !gamepad1.b) {
            rightClaw.setPosition(0.57); // Right claw closed
            leftClaw.setPosition(0.57); // Left claw closed
        }
        if (gamepad1.b) {
            rightClaw.setPosition(0.44); // Right claw open
            leftClaw.setPosition(0.7); // Left claw open

        }

    }
}
