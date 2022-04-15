package org.firstinspires.ftc.teamcode.competition.auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.types.Color;
import org.firstinspires.ftc.teamcode.competition.types.ElementPosition;
import org.firstinspires.ftc.teamcode.competition.types.ParkingMethod;
import org.firstinspires.ftc.teamcode.competition.types.StartPosition;
import org.firstinspires.ftc.teamcode.competition.util.AutoState;
import org.firstinspires.ftc.teamcode.competition.util.BlueTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.competition.util.Constants;
import org.firstinspires.ftc.teamcode.competition.util.Hardware22;
import org.firstinspires.ftc.teamcode.competition.util.RedTrajectoryGenerator;
import org.firstinspires.ftc.teamcode.competition.util.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import kotlin.NotImplementedError;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(name="Auto final", preselectTeleOp="TeleOp final blue")
public class AutoFinal extends LinearOpMode {
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    //private static int valLeft = -1;
    private static int valRight = -1;
    //Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware22 robot = new Hardware22(hardwareMap);
        SampleMecanumDrive drive = robot.drive;

        OpenCvCamera webcam = initCamera();
        initMotors(robot);
        int liftEncoderStart = robot.liftMotor.getCurrentPosition();


        AutoState state = getUserInput();
        Color color = state.color;
        StartPosition startPosition = state.position;
        ParkingMethod parkingMethod = state.parkMethod;
        long delay = state.delay;

        double towerSpeed = Constants.towerWheelSpeedAuto;
        if (color == Color.RED) {
            towerSpeed = -towerSpeed;
        }

        ArrayList<ArrayList<Trajectory>> trajs;

        trajs = generateTrajs(drive, color, startPosition, parkingMethod);

        // Confirm auto settings
        telemetry.addLine("Init complete, ready to run");
        telemetry.addData("Color", color);
        telemetry.addData("Start pos", startPosition);
        telemetry.addData("Parking method", parkingMethod);
        telemetry.addData("Delay", delay);
        telemetry.update();

        if (isStopRequested()) { return; }

        waitForStart();

        // Has to be initted here and not before as we cannot extend 18 inch bounding box before
        // game starts
        robot.tseServo.setPosition(Constants.tseArmInitPosition);

        long startTime = System.nanoTime();

        sleep(delay);

        ElementPosition elementPosition = determineDuckPosition();

        //added this here to try to fix lifter in autonomous
        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Traj 1");
        telemetry.addData("Position", elementPosition);
        telemetry.update();

        TrajectoryGenerator.executeTrajectoryList(drive, trajs.get(0)); // going to shipping hub

        telemetry.addLine("Traj 2");
        telemetry.update();

        dumpPreloaded(robot, liftEncoderStart, elementPosition);

        if (startPosition == StartPosition.FRONT) {
            TrajectoryGenerator.executeTrajectoryList(drive, trajs.get(1)); // going to duck wheel
            sleep(300);
            robot.towerMotor.setPower(towerSpeed);
            sleep(3000);
            robot.towerMotor.setPower(0);
        }

        telemetry.addLine("Traj 3");
        telemetry.update();
        TrajectoryGenerator.executeTrajectoryList(drive, trajs.get(2)); // going to park in warehouse

        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        // This dumbness is b/c java doesn't provide an easy way to round to a certain number of decimal places
        double durationSeconds = Math.round((duration / (double) Math.pow(10,9)) * 100.0) / 100.0;

        telemetry.addData("Time Elapsed", durationSeconds);
        telemetry.update();

        sleep(2000);
    }

    private ArrayList<ArrayList<Trajectory>> generateTrajs(SampleMecanumDrive drive, Color color, StartPosition startPosition, ParkingMethod parkingMethod) {
        ArrayList<ArrayList<Trajectory>> trajs;
        if (color == Color.RED) {
            RedTrajectoryGenerator generator = new RedTrajectoryGenerator(drive, startPosition, parkingMethod);
            trajs = generator.generateTrajectories();
        } else {
            BlueTrajectoryGenerator generator = new BlueTrajectoryGenerator(drive, startPosition, parkingMethod);
            trajs = generator.generateTrajectories();
        }
        return trajs;
    }

    private void initMotors(Hardware22 robot) {
        //initialize motors
        robot.dumpServo.setPosition(Constants.collectPosition);

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setPower(1.0);
        robot.liftMotor.setPower(0.0);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //set start position for linear slide encoder
        //robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private OpenCvCamera initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(new SamplePipeline());
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        return webcam;

        //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
        //If this line is uncommented you will not recieve telemetry

//        telemetry.addData("Values", valLeft+"   "+valMid);
//        telemetry.update();

        //code needed for camera to display on FTC Dashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);
//        telemetry.update();
        //this is next level. ms weyrens, that's irresponsible
    }

    private void dumpPreloaded(Hardware22 robot, int liftEncoderStart, ElementPosition elementPosition) {
        // Encoder Counts, Bottom: 1100
        // Middle 2170
        // Top 3470

        int encTarget;
        if (elementPosition == ElementPosition.RIGHT) {
            encTarget = Constants.highEncoder;
            telemetry.addLine("Dump top");
        } else if (elementPosition == ElementPosition.MIDDLE) {
            encTarget = Constants.midEncoder;
            telemetry.addLine("Dump middle");
        } else {
            encTarget = Constants.lowEncoder;
            telemetry.addLine("Dump bottom");
        }

        telemetry.addData("Encoder Target", encTarget);
        telemetry.addData("Encoder", robot.liftMotor.getCurrentPosition());
        telemetry.update();

        /*--------------------GLITCH REPAIR (DO NOT REMOVE)------------------------------*/
        // run the encoder 2 encoder counts (doesnt work) but gets us through the glitch :)
        int encoderGlitchTarget;
        encoderGlitchTarget = liftEncoderStart + 2;

        robot.liftMotor.setTargetPosition(encoderGlitchTarget);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(0.2);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
            //robot.liftMotor.setPower(0.2);
            telemetry.addData("Encoder", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Encoder target",encoderGlitchTarget);

            telemetry.update();
        }

        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*--------------------GLITCH REPAIR (DO NOT REMOVE)------------------------------*/

        int encoderUpTarget;
        encoderUpTarget = liftEncoderStart + encTarget;

        robot.liftMotor.setTargetPosition(encoderUpTarget);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(1.0);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
            //robot.liftMotor.setPower(1.0);
            telemetry.addData("Encoder", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Encoder target",encoderUpTarget);

            telemetry.update();
        }

        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
        robot.dumpServo.setPosition(Constants.dumpPosition);
        sleep(1000);
        robot.dumpServo.setPosition(Constants.collectPosition);
        sleep(500);

        int encoderDownTarget;
        encoderDownTarget = liftEncoderStart;

        robot.liftMotor.setTargetPosition(encoderDownTarget);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(1.0);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
            //robot.liftMotor.setPower(1.0);
            telemetry.addData("Encoder", robot.liftMotor.getCurrentPosition());
            telemetry.addData("Encoder target",encoderDownTarget);
            telemetry.update();
        }

        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sleep for timing consistency. Moved here because it used to be before slide raised and that
        // caused significant distress for the team.
        if (elementPosition == ElementPosition.MIDDLE) {
            sleep(1000);
            encTarget = Constants.midEncoder;
        } else {
            encTarget = Constants.lowEncoder;
        }
    }

    private ElementPosition determineDuckPosition() {
        ElementPosition elementPosition = null;
        if (valRight == 255) {
            telemetry.addData("Position", "Right");
            telemetry.update();
            elementPosition = ElementPosition.RIGHT;
            sleep(200);
        } else if (valMid == 255) {
            telemetry.addData("Position", "Middle");
            telemetry.update();
            elementPosition = ElementPosition.MIDDLE;
            sleep(200);
        } else {
            telemetry.addData("Position", "Left");
            telemetry.update();
            elementPosition = ElementPosition.LEFT;
            sleep(200);
        }

        return elementPosition;
    }

    public AutoState getUserInput() {
        gamepad2.reset();

        Color color = getColor();
        sleep(100);

        StartPosition startPosition = getPosition();
        sleep(100);

        ParkingMethod parkingMethod = getParkMethod(startPosition);
        sleep(100);

        long delay = getDelay();
        sleep(100);

        confirmAutoGood();

        return new AutoState(color, startPosition, parkingMethod, delay);
    }

    private void confirmAutoGood() {
        // Prompt users to check if auto is good
        telemetry.addLine("Did you check camera stream? Press b or Circle");
        telemetry.update();
        while (!isStopRequested() && !gamepad2.b) {}

        telemetry.addLine("Did you preload a box? Press a or X");
        telemetry.update();
        while (!isStopRequested() && !gamepad2.a) {}

        telemetry.addLine("Are the linear slide and TSE arm down? Press x or Square");
        telemetry.update();
        while (!isStopRequested() && !gamepad2.x) {}
    }

    private long getDelay() {
        boolean buttonUnpressed = true;
        long delay = 0;

        while (!isStopRequested()) {
            gamepad2.toString();
            telemetry.addData("Delay? RB to add 10 ms, LB to add 100ms, y or Triangle done", delay);
            telemetry.update();
            if (gamepad2.right_bumper && buttonUnpressed) {
                delay += 10;
                buttonUnpressed = false;
            } else if (gamepad2.left_bumper && buttonUnpressed) {
                delay += 100;
                buttonUnpressed = false;
            } else if (gamepad2.y) {
                break;
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                buttonUnpressed = true;
            }
        }
        telemetry.addLine("Delay confirmed, " + delay);
        telemetry.update();
        return delay;
    }

    private ParkingMethod getParkMethod(StartPosition startPosition) {
        telemetry.addLine("Parking Method?");
        telemetry.addLine("Wall = dpad Right");
        telemetry.addLine("Barrier = dpad Left");
        telemetry.addLine("Storage = gamepad a or X");
        telemetry.update();

        ParkingMethod parkingMethod = null;

        while (!isStopRequested()) {
            gamepad2.toString();
            if (gamepad2.dpad_left) {
                parkingMethod = ParkingMethod.BARRIER;
                break;
            } else if (gamepad2.dpad_right) {
                parkingMethod = ParkingMethod.WALL;
                break;
            } else if (gamepad2.a) {
                if (startPosition == StartPosition.BACK) {
                    throw new NotImplementedError("This route doesn't exist moron");
                }
                parkingMethod = ParkingMethod.STORAGE;
                break;
            }
        }

        telemetry.addLine("Parking method confirmed, " + parkingMethod);
        telemetry.update();

        return parkingMethod;
    }

    private StartPosition getPosition() {
        telemetry.addLine("Position");
        telemetry.addLine("Front = dpad Up, no delay time");
        telemetry.addLine("Back = dpad Down");
        telemetry.update();

        StartPosition startPosition = null;

        while (!isStopRequested()) {
            gamepad2.toString();
            if (gamepad2.dpad_up) {
                startPosition = StartPosition.FRONT;
                break;
            } else if (gamepad2.dpad_down) {
                startPosition = StartPosition.BACK;
                break;
            }
        }

        telemetry.addLine("Position confirmed, " + startPosition);
        telemetry.update();

        return startPosition;
    }

    private Color getColor() {
        telemetry.addLine("Color?");
        telemetry.addLine("Blue = X or Square");
        telemetry.addLine("Red = B or Circle");
        telemetry.update();

        Color color = null;

        while (!isStopRequested()) {
            // This is for some reason required as otherwise we can only run autonomous once
            // before we need to restart the robot. Included at the beginning of each while loop
            gamepad2.toString();
            if (gamepad2.x) {
                color = Color.BLUE;
                break;
            } else if (gamepad2.b) {
                color = Color.RED;
                break;
            }
        }
        telemetry.addLine("Color confirmed, " + color);
        telemetry.update();

        return color;
    }


    static class SamplePipeline extends OpenCvPipeline {
        Mat yCbCr = new Mat();
        Mat yMat = new Mat();
        Mat CbMat = new Mat();
        Mat CrMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        private static final float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        private static final float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        private static final float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        private static final float[] midPos = {3f/8f+offsetX, 4f/8f+offsetY};
        //  private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};
        //moves all rectangles right or left by amount. units are in ratio to monitor

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCr, yMat, 0);//extracts cb channel as black and white RGB
            Core.extractChannel(yCbCr, CrMat, 1);//extracts cb channel as black and white RGB
            Core.extractChannel(yCbCr, CbMat, 2);//extracts cb channel as black and white RGB
            Imgproc.threshold(CbMat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            //any pixel with a hue value less than 102 is being set to 0 (yellow)
            //any pixel with a hue value greater than 102 is being set to 255(blue)
            //Then swaps the blue and the yellows with the binary inv line
            CbMat.copyTo(all);//copies mat object

            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            //double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            //valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            //Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            //Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            /*
            Imgproc.rectangle(//1-3

                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
                    */
            float rectHeight = 1f / 8f;
            float rectWidth = 1f / 8f;
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]- rectWidth /2),
                            input.rows()*(midPos[1]- rectHeight /2)),
                    new Point(
                            input.cols()*(midPos[0]+ rectWidth /2),
                            input.rows()*(midPos[1]+ rectHeight /2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]- rectWidth /2),
                            input.rows()*(rightPos[1]- rectHeight /2)),
                    new Point(
                            input.cols()*(rightPos[0]+ rectWidth /2),
                            input.rows()*(rightPos[1]+ rectHeight /2)),
                    new Scalar(0, 255, 0), 3);


            //return input; // this is the line that declares which image is returned to the viewport (DS)
            //return CbMat;
            return all;
        }
    }
}