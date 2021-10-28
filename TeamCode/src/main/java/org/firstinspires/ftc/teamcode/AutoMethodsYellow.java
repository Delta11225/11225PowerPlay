package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


/**
 * Created by robotics on 10/12/2017.
 */

public abstract class AutoMethodsYellow extends LinearOpMode {

    /* Declare OpMode members. */

    double counts_per_rev = 537.6;
    double counts_per_rev_arm = 1993.6;
    double actual_speed = 0.6;
    double wheelDiameter_cm = 10;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    double widthOfRobot_cm = 40;
    double lengthOfRobot_cm = 33.5;

    double radiusOfRobot_cm = Math.sqrt(((.5*widthOfRobot_cm)*(0.5*widthOfRobot_cm)) + ((0.5*lengthOfRobot_cm)*(0.5*lengthOfRobot_cm)));
    double circumferenceOfRobotCircle_cm = 2 * 3.1415 * radiusOfRobot_cm;


    double counts_per_cm = counts_per_rev * DRIVE_GEAR_REDUCTION / (wheelDiameter_cm * 3.1415);
    double counts_per_degree = DRIVE_GEAR_REDUCTION * counts_per_rev_arm / (360);




    DigitalChannel digitalTouch;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;



    HunterHardware robot = new HunterHardware();
    private ElapsedTime runtime = new ElapsedTime();

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    private boolean targetVisible = false;
    String vuforiaCode = "";






    ////METHODS////
    ///Vuforia/////////////////////////////////////////

    ////Vuforia Code////
    public void vuforiaCode(double time) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWAQ1WL/////AAAAGXVdDwxhtUkZvdHHgJOEVsNIDr+6XlP2GxHyPxlCyj6GvdGDrT75jhVpmYXcXxWlLdDukO49wWW1CWWqMcE12j3OVWX9aA8ayXr50unvfcKlIbRDQDNEfCOArmADXfDcoW22JaHvoD4hRhQp6umyV1Av/ceiMWvCETajTt/TebeJMud4EBSm5eyPNKTEabVLoGP9PEUHzC2zD7NziZiQBkQaYa4NpIRgwMzz24E6qnz3mVO4jjLPlwHuzkTBu9/YZvmhdx7dGHCPxl100vjGSlPunKtqJOz679vk0r0T8u/TEdntEIbaQ7rHPXJ57lXaBvOf1aMK4Wk5EEJYTPYkCVi0hxvnTHbkMTKEgGoW/hM8";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        telemetry.update();
        waitForStart();

        targetsSkyStone.activate();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    //////CODE WEYRENS ADDED/////
                    if(trackable.getName().equals("Stone Target")){
                        telemetry.addLine("Stone Target Visable!!");
                    }
                    //////////////////////////////////
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            String positionSkystone = ""; //Weyrens Added
            double yPosition = 10000;
            double xPosition = 10000;
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));
                //////CODE WEYRENS ADDED/////
                yPosition = translation.get(1);

                //values for when camera is 50 cm from stones in landscape mode (camera on left)
                if(yPosition < -150){
                    vuforiaCode = "left";
                    telemetry.addLine("code = left");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);
                }
                else if (yPosition > 150){
                    vuforiaCode = "right";
                    telemetry.addLine("code = right");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);
                }
                else
                    vuforiaCode = "center";
                    telemetry.addLine("code = center");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);

                //////////////////////////////////
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }

        }

    }

    //////////////Collector On/////////////////////
    public void collectorIn() {
        robot.init(hardwareMap);
        robot.collectorLeft.setPower(-1);
        robot.collectorRight.setPower(1);
    }
    public void collectorOut() {
        robot.init(hardwareMap);
        robot.collectorLeft.setPower(1);
        robot.collectorRight.setPower(-1);
    }
    public void collectorOff() {
        robot.init(hardwareMap);
        robot.collectorLeft.setPower(0);
        robot.collectorRight.setPower(0);
    }
    ///Encoder///
    ///Reset Encoder///
    public void resetEncoder() {
        //This is what should be in here//

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Encoders Reset");    //
        telemetry.update();


    }

    ///Straight and dump capstone///
    public void encoderDrive_Straight_Dump(double speed,
                                      double distanceCM,
                                      double timeOut) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;
        int loopCount = 0;

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armMotor.setDirection(DcMotor.Direction.FORWARD);

            runtime.reset();
            robot.armMotor.setPower(.5);
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.rearRight.setPower(Math.abs(speed));
            robot.rearLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                // Display it for the driver.
                if (loopCount == 0)
                {
                    telemetry.addLine("in loop");
                    telemetry.update();
                    //robot.armMotor.setPower(.225);
                    //collectorIn();
                    sleep(1000); //determine time at speed
                    //robot.armServo.setPosition(.75);
                    //sleep(200);
                    robot.armMotor.setPower(-.225);
                    sleep(500); //determine time at speed
                    robot.armMotor.setPower(0);
                }
                else{}

                loopCount = loopCount + 1;


                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", NEW_upright_target, NEW_upleft_target, NEW_rearleft_target, NEW_rearright_target);
                telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.rearRight.getCurrentPosition(),
                        robot.rearLeft.getCurrentPosition());
                telemetry.addData("Runtime", "%.2f", runtime.seconds());
                telemetry.update();
            }

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD);
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    }



    ///Straight///

    public void encoderDrive_Straight(double speed,
                                      double distanceCM,
                                      double timeOut) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.rearRight.setPower(Math.abs(speed));
            robot.rearLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", NEW_upright_target, NEW_upleft_target, NEW_rearleft_target, NEW_rearright_target);
                telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.rearRight.getCurrentPosition(),
                        robot.rearLeft.getCurrentPosition());
                telemetry.addData("Runtime", "%.2f", runtime.seconds());
                telemetry.update();
            }

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD);
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    }

    public void encoderDrive_Straight_Acc(double speed, double distanceCM,double rampupPercentage, double timeOut, double slowPercentage) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        int FRnewzero = robot.frontRight.getCurrentPosition();
        int FLnewzero = robot.frontRight.getCurrentPosition();
        int RRnewzero = robot.frontRight.getCurrentPosition();
        int RLnewzero = robot.frontRight.getCurrentPosition();

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            double FRstartEncoder = robot.frontRight.getCurrentPosition();

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();


            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                double distance = Math.abs(distanceCM*counts_per_cm);
                double slowdistance = Math.abs(slowPercentage*distance);
                double remainder = (Math.abs(robot.frontRight.getCurrentPosition() - NEW_upright_target));
                double rampdistance = Math.abs(rampupPercentage*distance);
                double distancetraveled = Math.abs(robot.frontRight.getCurrentPosition()- FRstartEncoder);


                if (distancetraveled < rampdistance ){
                    telemetry.addLine("speeding up");
                    telemetry.addData("distance", distance);
                    telemetry.addData("initial encoder value", FRstartEncoder);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("ramp distance", rampdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    double rampRatio = distancetraveled/rampdistance;
                    robot.frontRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.frontLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));

                }
/*
                else if (distancetraveled < (distance - slowdistance)){//&&(remainderFL > slowPercentage*distance)&&(remainderRR > slowPercentage*distance)&&(remainderRL > slowPercentage*distance)){
                    //This keep the robot moving at full speed until the robot is 20% away from its target
                    telemetry.addLine("max speed");
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("slow distance", slowdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    robot.frontRight.setPower(speed);
                    robot.frontLeft.setPower(speed);
                    robot.rearRight.setPower(speed);
                    robot.rearLeft.setPower(speed);

                }

*/
                else {
                    //This code will slow down the motor as it approaches the target.
                    // The distance variable represents the total number of ticks for a specific movement.
                    // The motors will slow down to 10% of the max speed gradually as the target is approached.

                    double slowRatio = (distance - distancetraveled)/slowdistance;
                    telemetry.addLine("slowing down");
                    telemetry.addData("distance", distance);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("slow distance", slowdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.addData("slow ratio", slowRatio);
                    telemetry.update();

                    robot.frontRight.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.frontLeft.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.rearRight.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.rearLeft.setPower(Range.clip(slowRatio,0.1,speed));
                }
            }
            telemetry.addLine("stop");
            telemetry.update();
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD);
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    }
/////////////////Side Acc////////////////////
    public void encoderDrive_Side_Acc(double speed, double distanceCM,double rampupPercentage, double timeOut, double slowPercentage) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        int FRnewzero = robot.frontRight.getCurrentPosition();
        int FLnewzero = robot.frontRight.getCurrentPosition();
        int RRnewzero = robot.frontRight.getCurrentPosition();
        int RLnewzero = robot.frontRight.getCurrentPosition();

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            double FRstartEncoder = robot.frontRight.getCurrentPosition();

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();


            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                double distance = Math.abs(distanceCM*counts_per_cm);
                double slowdistance = Math.abs(slowPercentage*distance);
                double remainder = (Math.abs(robot.frontRight.getCurrentPosition() - NEW_upright_target));
                double rampdistance = Math.abs(rampupPercentage*distance);
                double distancetraveled = Math.abs(robot.frontRight.getCurrentPosition()- FRstartEncoder);


                if (distancetraveled < rampdistance ){
                    telemetry.addLine("speeding up");
                    telemetry.addData("distance", distance);
                    telemetry.addData("initial encoder value", FRstartEncoder);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("ramp distance", rampdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    double rampRatio = distancetraveled/rampdistance;
                    robot.frontRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.frontLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));

                }
/*
                else if (distancetraveled < (distance - slowdistance)){//&&(remainderFL > slowPercentage*distance)&&(remainderRR > slowPercentage*distance)&&(remainderRL > slowPercentage*distance)){
                    //This keep the robot moving at full speed until the robot is 20% away from its target
                    telemetry.addLine("max speed");
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("slow distance", slowdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    robot.frontRight.setPower(speed);
                    robot.frontLeft.setPower(speed);
                    robot.rearRight.setPower(speed);
                    robot.rearLeft.setPower(speed);

                }

*/
                else {
                    //This code will slow down the motor as it approaches the target.
                    // The distance variable represents the total number of ticks for a specific movement.
                    // The motors will slow down to 10% of the max speed gradually as the target is approached.

                    double slowRatio = (distance - distancetraveled)/slowdistance;
                    telemetry.addLine("slowing down");
                    telemetry.addData("distance", distance);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("slow distance", slowdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.addData("slow ratio", slowRatio);
                    telemetry.update();

                    robot.frontRight.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.frontLeft.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.rearRight.setPower(Range.clip(slowRatio,0.1,speed));
                    robot.rearLeft.setPower(Range.clip(slowRatio,0.1,speed));
                }
            }
            telemetry.addLine("stop");
            telemetry.update();
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD);
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    }
//////////////////Red foundation ending///////////////
public void RedFoundationEnding() {

    robot.rightServo.setPosition(.5);
    robot.leftServo.setPosition(.5);
    sleep(300);
    encoderDrive_Straight(1, 55, 5);
    encoderTurn_Acc("clockwise", 1, 190, 0.3,5,.3);
    encoderDrive_Straight(1, -22, 5);
    robot.rightServo.setPosition(0);
    robot.leftServo.setPosition(1);
    encoderDrive_Straight(1, 90, 5);
}
    //////////////////Blue foundation ending///////////////
    public void BlueFoundationEnding() {

        robot.rightServo.setPosition(.5);
        robot.leftServo.setPosition(.5);
        sleep(300);
        encoderDrive_Straight(1, 45, 5);
        encoderTurn_Acc("counterclockwise", 1, 160, 0.3,5,.3);
        encoderDrive_Straight(1, -22, 5);
        robot.rightServo.setPosition(0);
        robot.leftServo.setPosition(1);
        encoderDrive_Straight(1, 90, 5);
    }
//////////////////Diagonal Move/////////////////////
    public void diagonalBackRight(double time) {
        runtime.reset();
        while (opModeIsActive() &&
            (runtime.seconds() < time)){
        robot.init(hardwareMap);
        robot.rearLeft.setPower(-1);
        robot.frontRight.setPower(-1);
        }
        robot.rearLeft.setPower(0);
        robot.frontRight.setPower(0);
    }

    public void diagonalBackLeft(double time) {
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < time)){
            robot.init(hardwareMap);
            robot.rearRight.setPower(-1);
            robot.frontLeft.setPower(-1);
        }
        robot.rearLeft.setPower(0);
        robot.frontRight.setPower(0);
    }
    public void diagonalBackLeftSteep(double time) {
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < time)){
            robot.init(hardwareMap);
            robot.rearRight.setPower(-1);
            robot.frontLeft.setPower(-1);
            robot.frontRight.setPower(-0.15);
            robot.rearLeft.setPower(-0.15);
        }
        robot.rearLeft.setPower(0);
        robot.frontRight.setPower(0);
    }
    ///Side///
    public void encoderDrive_Side(double speed,
                                      double distanceCM,
                                      double timeOut) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (distanceCM * counts_per_cm);
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.rearRight.setPower(Math.abs(speed));
            robot.rearLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", NEW_upright_target, NEW_upleft_target, NEW_rearleft_target, NEW_rearright_target);
                telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.rearRight.getCurrentPosition(),
                        robot.rearLeft.getCurrentPosition());
                telemetry.addData("Runtime", "%.2f", runtime.seconds());
                telemetry.update();
            }

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontRight.setDirection(DcMotor.Direction.FORWARD);
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD);
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);
        }

    }
    public void encoder_ArmRotate(double speed,
                                        double degrees,
                                        String direction,
                                        double timeOut) {

        robot.init(hardwareMap);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        robot.armMotor.setDirection(DcMotor.Direction.FORWARD);

        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        if (opModeIsActive()) {

            NEW_upright_target = robot.armMotor.getCurrentPosition() + (int) (degrees * counts_per_degree);



            // Turn On RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            if (direction == "down"){
                robot.armMotor.setTargetPosition(NEW_upright_target*1);
            } else if (direction == "up"){
                robot.armMotor.setTargetPosition(NEW_upright_target*-1);
            }
            robot.armMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.armMotor.isBusy())) {
            }

            robot.armMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
//PIVOT TURN TOTAL/////////////////
    public void pivotTurn(double angle, double speed, double degreeTolerance) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        robot.init(hardwareMap);
        telemetry.update();

        double difference = (angle - angles.firstAngle);
        if (difference < 0) {//if difference is negative
            difference = difference + 360;
        }
        if (difference < 180) {
            CCWTurnPoint(angle, speed, degreeTolerance);
        } else {
            CWTurnPoint(angle, speed, degreeTolerance);
        }
    }

    //////CW Turn////////////////
    public void CWTurnPoint(double angle, double speed, double degreeTolerance) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        robot.init(hardwareMap);
        telemetry.addData("motor speed", "%.1f", speed);
        telemetry.update();

        double newAngle;

        if (angle >180) {
            newAngle = angle - 360;
        }
        else {
            newAngle = angle;
        }
        while ((Math.abs(newAngle - angles.firstAngle)) > degreeTolerance) {
            telemetry.update();
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.rearLeft.setPower(speed);
            robot.rearRight.setPower(-speed);
        }

        while (Math.abs(newAngle - angles.firstAngle) > 3) {
            telemetry.update();
            robot.frontLeft.setPower(0.13);
            robot.frontRight.setPower(-0.13);
            robot.rearLeft.setPower(0.13);
            robot.rearRight.setPower(-0.13);
        }

        telemetry.update();
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
        sleep(500);

    }
    public void CCWTurnPoint(double angle, double speed, double degreeTolerance) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
        robot.init(hardwareMap);

        telemetry.addData("motor speed", "%.1f", speed);
        telemetry.update();
        double newAngle = 0;

        if (angle >180) {
            newAngle = angle - 360;
        }
        if (angle <= 180) {
            newAngle = angle;
        }

        while ((Math.abs(newAngle - angles.firstAngle)) > degreeTolerance) {
            telemetry.addLine("Loop 1");
            telemetry.update();
            robot.frontLeft.setPower(-speed);
            robot.frontRight.setPower(speed);
            robot.rearLeft.setPower(-speed);
            robot.rearRight.setPower(speed);
        }

        while (Math.abs(newAngle - angles.firstAngle) > 3) {
            telemetry.addLine("Loop 2");
            telemetry.update();
            robot.frontLeft.setPower(-0.13);
            robot.frontRight.setPower(0.13);
            robot.rearLeft.setPower(-0.13);
            robot.rearRight.setPower(0.13);
        }

        telemetry.addLine("exit loop");
        telemetry.update();
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
        sleep(500);

    }
    public void foundationAttach() {
        robot.rightServo.setPosition(.4);
        robot.leftServo.setPosition(.6);
    }
    public void foundationRelease() {
        robot.rightServo.setPosition(0);
        robot.leftServo.setPosition(1);
    }
    ////////encoder turn method/////////////////////
    public void encoderTurn(String direction, double speed,
                            double degrees,
                            double timeOut) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        if (direction == "clockwise") {
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.FORWARD); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD); // reverse = forward
        }
        else if (direction == "counterclockwise"){
            robot.frontLeft.setDirection(DcMotor.Direction.REVERSE); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.REVERSE); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.REVERSE); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.REVERSE); // reverse = forward
        }


        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.rearRight.setPower(Math.abs(speed));
            robot.rearLeft.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", NEW_upright_target, NEW_upleft_target,
                        NEW_rearleft_target, NEW_rearright_target);
                telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                        robot.frontRight.getCurrentPosition(),
                        robot.frontLeft.getCurrentPosition(),
                        robot.rearRight.getCurrentPosition(),
                        robot.rearLeft.getCurrentPosition());
                telemetry.addData("Runtime", "%.2f", runtime.seconds());
                telemetry.update();
            }

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.FORWARD); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.REVERSE); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.REVERSE); // reverse = forward
        }

    }
    ////////encoder turn method with accelerations/////////////////////
    public void encoderTurn_Acc(String direction, double speed,
                            double degrees,double rampupPercentage,
                            double timeOut,double slowPercentage) {

        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.frontRight.getCurrentPosition(),
                robot.frontLeft.getCurrentPosition(),
                robot.rearRight.getCurrentPosition(),
                robot.rearLeft.getCurrentPosition());
        telemetry.update();

        if (direction == "clockwise") {
            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.FORWARD); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.FORWARD); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.FORWARD); // reverse = forward
        }
        else if (direction == "counterclockwise"){
            robot.frontLeft.setDirection(DcMotor.Direction.REVERSE); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.REVERSE); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.REVERSE); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.REVERSE); // reverse = forward
        }


        int NEW_upright_target;
        int NEW_upleft_target;
        int NEW_rearright_target;
        int NEW_rearleft_target;

        if (opModeIsActive()) {

            NEW_upright_target = robot.frontRight.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_upleft_target = robot.frontLeft.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_rearright_target = robot.rearRight.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
            NEW_rearleft_target = robot.rearLeft.getCurrentPosition() + (int) (((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));

            double FRstartEncoder = robot.frontRight.getCurrentPosition();

            robot.frontRight.setTargetPosition(NEW_upright_target);
            robot.frontLeft.setTargetPosition(NEW_upleft_target);
            robot.rearRight.setTargetPosition(NEW_rearright_target);
            robot.rearLeft.setTargetPosition(NEW_rearleft_target);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.rearRight.setPower(Math.abs(speed));
            robot.rearLeft.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.frontRight.isBusy() && robot.frontLeft.isBusy() && robot.rearLeft.isBusy() && robot.rearRight.isBusy())) {
                double distance = Math.abs(((degrees/360)*circumferenceOfRobotCircle_cm* counts_per_cm) * Math.sqrt(2));
                double slowdistance = Math.abs(slowPercentage*distance);
                double remainder = (Math.abs(robot.frontRight.getCurrentPosition() - NEW_upright_target));
                double rampdistance = Math.abs(rampupPercentage*distance);
                double distancetraveled = Math.abs(robot.frontRight.getCurrentPosition()- FRstartEncoder);


                if (distancetraveled < rampdistance ){
                    telemetry.addLine("speeding up");
                    telemetry.addData("distance", distance);
                    telemetry.addData("initial encoder value", FRstartEncoder);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("ramp distance", rampdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    double rampRatio = distancetraveled/rampdistance;
                    robot.frontRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.frontLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearRight.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));
                    robot.rearLeft.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));

                }

                else {
                    //This code will slow down the motor as it approaches the target.
                    // The distance variable represents the total number of ticks for a specific movement.
                    // The motors will slow down to 10% of the max speed gradually as the target is approached.

                    double slowRatio = (distance - distancetraveled)/slowdistance;
                    telemetry.addLine("slowing down");
                    telemetry.addData("distance", distance);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.frontRight.getCurrentPosition());
                    telemetry.addData("slow distance", slowdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.addData("slow ratio", slowRatio);
                    telemetry.update();

                    robot.frontRight.setPower(Range.clip(slowRatio,0.2,speed));
                    robot.frontLeft.setPower(Range.clip(slowRatio,0.2,speed));
                    robot.rearRight.setPower(Range.clip(slowRatio,0.2,speed));
                    robot.rearLeft.setPower(Range.clip(slowRatio,0.2,speed));
                }
            }
            telemetry.addLine("stop");
            telemetry.update();
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.rearLeft.setPower(0);



            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.frontLeft.setDirection(DcMotor.Direction.FORWARD); //forward = forward
            robot.frontRight.setDirection(DcMotor.Direction.FORWARD); //reverse = forward
            robot.rearLeft.setDirection(DcMotor.Direction.REVERSE); // forward = forward
            robot.rearRight.setDirection(DcMotor.Direction.REVERSE); // reverse = forward
        }

    }

//----------------------------------------------------------------------------------------------
// Telemetry for IMU
//----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });


    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
















