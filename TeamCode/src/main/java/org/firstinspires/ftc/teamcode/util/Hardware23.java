/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.nulldevices.NullColorSensor;
import org.firstinspires.ftc.teamcode.util.nulldevices.NullDcMotor;
import org.firstinspires.ftc.teamcode.util.nulldevices.NullDrive;
import org.firstinspires.ftc.teamcode.util.nulldevices.NullServo;
import org.firstinspires.ftc.teamcode.util.nulldevices.NullWebcamName;

/**
 * This class holds all of the robot's hardware in one spot.
 *
 * PLEASE follow these instructions when adding new hardware:
 * IF you are TESTING, please wrap each individual call to the hardware map in a try-catch block,
 * IGNORING any errors. This prevents robot total failure even if some hardware is disconnected.
 * REMOVE THE TRY CATCH BLOCKS BEFORE COMPETITION.
 * ENSURE each INDIVIDUAL CALL to the hardware map is in its OWN try-catch block.
 */
public class Hardware23 {

    public DcMotor rearLeft;
    public DcMotor rearRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public WebcamName logitechWebcam;

    public DcMotor linearSlide;
    public Servo rightClaw;
    public Servo leftClaw;

    public ColorSensor colorSensor;
    // RoadRunner drive class
    public SampleMecanumDrive drive;

    /**
     * Gets all appropriate hardware from hardware map
     * @param hardwareMap The hardware map to get hardware devices from, as supplied by the opmode
     */
    public Hardware23(HardwareMap hardwareMap) {
        // For roadrunner
        drive = new SampleMecanumDrive(hardwareMap);

        // We wrap these in try catch blocks to prevent the robot from failing when something is unplugged
        // or not connected to the robot. Is not enabled for competition
        try {
            rearLeft = hardwareMap.dcMotor.get("rear_left");
//            rearLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            // If we are in competition mode we throw the error, otherwise we replace the device
            // with a "null" device, which basically just acts as a dummy device and eats all method calls
            // with no effect
            if (!Constants.COMPETITION_MODE) {
                rearLeft = new NullDcMotor();
            } else {
                throw e;
            }
        }

        try {
            frontLeft = hardwareMap.dcMotor.get("front_left");
//            frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                frontLeft = new NullDcMotor();
            } else {
                throw e;
            }
        }

        try {
            frontRight = hardwareMap.dcMotor.get("front_right");
//            frontRight.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                frontRight = new NullDcMotor();
            } else {
                throw e;
            }
        }

        try {
            rearRight = hardwareMap.dcMotor.get("rear_right");
//            rearRight.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                rearRight = new NullDcMotor();
            } else {
                throw e;
            }
        }

        try {
            logitechWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                logitechWebcam = new NullWebcamName();
            } else {
                throw e;
            }
        }

        try {
            linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                linearSlide = new NullDcMotor();
            } else {
                throw e;
            }
        }

        try {
            rightClaw = hardwareMap.get(Servo.class, "right_claw");
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                rightClaw = new NullServo();
            } else {
                throw e;
            }
        }

        try {
            leftClaw = hardwareMap.get(Servo.class, "left_claw");
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                leftClaw = new NullServo();
            } else {
                throw e;
            }
        }

        try {
            colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        } catch (Exception e) {
            if (!Constants.COMPETITION_MODE) {
                colorSensor = new NullColorSensor();
            } else {
                throw e;
            }
        }

        try {
            drive = new SampleMecanumDrive(hardwareMap);
        } catch (Exception e) {
            if (Constants.COMPETITION_MODE) {
                throw e;
            }
        }
    }
}