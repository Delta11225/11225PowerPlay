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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.competition.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * PLEASE follow these instructions when adding new hardware:
 * IF you are TESTING, please wrap each individual call to the hardware map in a try-catch block,
 * IGNORING any errors. This prevents robot total failiure even if some hardware is disconnected.
 * REMOVE THE TRY CATCH BLOCKS BEFORE COMPETITION.
 * ENSURE each INDIVIDUAL CALL to the hardware map is in its OWN try-catch block.
 */
public class Hardware22 {

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public WebcamName logitechWebcam = null;
    public DcMotor towerMotor = null;
    public Servo dumpServo = null;
    public Servo tseServo = null;
    public DcMotor collectionMotor = null;
    public DcMotor liftMotor = null;


    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    private final HardwareMap hwMap = null;
    private final ElapsedTime runtime = new ElapsedTime();

    // RoadRunner driver
    public SampleMecanumDrive drive;
    public TrajectoryGenerator generator;


    public Hardware22(HardwareMap hardwareMap) {
        // Define and initialize motors
        // NEVER DO THIS
//        try {
        rearLeft = hardwareMap.dcMotor.get("rear_left");
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
//        } catch (Exception ignored) {
//        }

//        try {
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        } catch (Exception ignored) {
//        }

//        try {
        frontRight = hardwareMap.dcMotor.get("front_right");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        } catch (Exception ignored) {
//        }

//        try {
        rearRight = hardwareMap.dcMotor.get("rear_right");
        rearRight.setDirection(DcMotor.Direction.FORWARD);
//        } catch (Exception ignored) {
//        }

//        try {
        logitechWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        } catch (Exception ignored) {
//        }

//        try {
        towerMotor = hardwareMap.dcMotor.get("tower_motor");
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        } catch (Exception ignored) {
//        }

//        try {
        dumpServo = hardwareMap.servo.get("servo_dump");
//        } catch (Exception ignored) {
//        }

//        try {
        tseServo = hardwareMap.servo.get("tse_servo");
//        } catch (Exception ignored) {
//        }


//        try {
        collectionMotor = hardwareMap.dcMotor.get("collection_motor");
//        } catch (Exception ignored) {
//        }

//        try {
        liftMotor = hardwareMap.dcMotor.get("lift_motor");
//        } catch (Exception ignored) {
//        }

//        try {
        drive = new SampleMecanumDrive(hardwareMap);
//        } catch (Exception ignored) {
//        }

//        try {
        generator = new TrajectoryGenerator(drive);
//        } catch (Exception ignored) {
//        }
    }
}








