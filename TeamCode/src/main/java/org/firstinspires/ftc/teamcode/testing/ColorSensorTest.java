package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTest extends OpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void init() {
        sensorColor = hardwareMap.get(ColorSensor.class, "colordistance_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"colordistance_sensor");
    }

    @Override
    public void loop() {
        if (sensorColor.red() > sensorColor.blue()) {
            telemetry.addLine("Red");
        }
        if (sensorColor.blue() > sensorColor.red()) {
            telemetry.addLine("Blue");
        }
        if (sensorDistance.getDistance(DistanceUnit.CM) < 3) {
            telemetry.addLine("Ready to grab");
        }


       // telemetry.addData("Color vals, r", sensorColor.red());
       // telemetry.addData("Color vals, g", sensorColor.green());
       // telemetry.addData("Color vals, b", sensorColor.blue());
       // telemetry.addData("Distance(cm)", sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
