package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@TeleOp
//@Disabled
public class ColorSensorTest extends OpMode {
    ColorSensor sensorColor;
    Hardware23 robot;

    @Override
    public void init() {
        robot = new Hardware23(hardwareMap);
        sensorColor = robot.colorSensor;
//        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
//        sensorDistance = hardwareMap.get(DistanceSensor.class,"color_sensor");
    }

    @Override
    public void loop() {
        if (sensorColor.red() > sensorColor.blue()) {
            telemetry.addLine("Red");
        }
        if (sensorColor.blue() > sensorColor.red()) {
            telemetry.addLine("Blue");
        }
        if (((DistanceSensor) sensorColor).getDistance(DistanceUnit.CM) <3) {
            telemetry.addLine("Ready to grab");
            robot.rightClaw.setPosition(Constants.rightClawClosed);
            robot.leftClaw.setPosition(Constants.leftClawClosed);
        } else {
            robot.rightClaw.setPosition(Constants.rightClawOpen);
            robot.leftClaw.setPosition(Constants.leftClawOpen);
        }


       // telemetry.addData("Color vals, r", sensorColor.red());
       // telemetry.addData("Color vals, g", sensorColor.green());
       // telemetry.addData("Color vals, b", sensorColor.blue());
       // telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
