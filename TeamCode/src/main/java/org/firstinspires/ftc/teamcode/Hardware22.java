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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.testing.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

public class Hardware22 {

//    private Telemetry telemetry;
//    public void init(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }


    /* Public OpMode members. */

    public DcMotor rearLeft = null;
    public DcMotor rearRight = null;
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public WebcamName logitechWebcam = null;
    public DcMotor towerMotor = null;
    public Servo dumpServo = null;
    public DcMotor collectionMotor = null;
    public DcMotor liftMotor = null;


    //public com.qualcomm.robotcore.hardware.GyroSensor GyroSensor;
    //public ModernRoboticsI2cGyro   gyro;
    //public ColorSensor colorSensor;    // Hardware Device Object


    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;


    /* local OpMode members. */
    private final HardwareMap hwMap = null;
    private final ElapsedTime runtime = new ElapsedTime();

    // RoadRunner driver
    public SampleMecanumDrive drive;
    public TrajectoryGenerator generator;


    /* Constructor */
    public Hardware22(HardwareMap hardwareMap) {
        // Define and initialize motors
        // NEVER DO THIS
        // FIXME rollback when done
        try {
            rearLeft = hardwareMap.dcMotor.get("rear_left");
            rearLeft.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
        }

        try {
            frontLeft = hardwareMap.dcMotor.get("front_left");
            frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        } catch (Exception ignored) {
        }

        try {
            frontRight = hardwareMap.dcMotor.get("front_right");
            frontRight.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ignored) {
        }

        try {
            rearRight = hardwareMap.dcMotor.get("rear_right");
            rearRight.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception ignored) {
        }

        try {
            logitechWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception ignored) {
        }

        try {
            towerMotor = hardwareMap.dcMotor.get("tower_motor");
            towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception ignored) {
        }

        try {
            dumpServo = hardwareMap.servo.get("servo_dump");
        } catch (Exception ignored) {
        }

        try {
            collectionMotor = hardwareMap.dcMotor.get("collection_motor");
        } catch (Exception ignored) {
        }

        try {
            liftMotor = hardwareMap.dcMotor.get("lift_motor");
        } catch (Exception ignored) {
        }

        drive = new SampleMecanumDrive(hardwareMap);
        generator = new TrajectoryGenerator(drive);

//        frontLeft = hwMap.dcMotor.get("front_left");
//        frontRight = hwMap.dcMotor.get("front_right");
//        rearRight = hwMap.dcMotor.get("rear_right");
//        logitechWebcam = hwMap.get(WebcamName .class, "Webcam 1");
//        towerMotor = hwMap.dcMotor.get ("tower_motor");
//        dumpServo = hwMap.servo.get("servo_dump");
//        collectionMotor = hwMap.dcMotor.get("collection_motor");
//        liftMotor = hwMap.dcMotor.get("lift_motor");

//            frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//            frontRight.setDirection(DcMotor.Direction.REVERSE);
//            rearLeft.setDirection(DcMotor.Direction.FORWARD);
//            rearRight.setDirection(DcMotor.Direction.REVERSE);

        //GyroSensor = hwMap.gyroSensor.get("gyro");
        //colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
        //towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}









