package org.firstinspires.ftc.teamcode.TeleOp1;


/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */



import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import org.firstinspires.ftc.robotcore.external.State;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Camera Imports

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

// Camera Imports


/*
 * This file includes a teleop (driver-controlled) file for the DinoByte! goBILDA® Robot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@Disabled
@TeleOp(name = "DB_Nov09_WebCam3", group = "teamcode")
//@Disabled
public class CameraTesting extends OpMode {

    //Camera Declarations

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 60.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    /*
    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel
    */

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    //Camera Declarations


    final double BOOTCLOSE_TIME_SECONDS = 0.75; //The boot servo waits this long before closing when a shot is requested.
    final double BOOTOPEN_TIME_SECONDS = 0.75; //The boot servo run this long before opening after making a shot.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1700; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1675; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 2025; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 2000; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double RTGATEOPEN_POSITION = 0.5; //the open and close position for the gate servos
    final double RTGATECLOSE_POSITION = 0.7;

    final double LFTGATEOPEN_POSITION = 0.5; //the open and close position for the gate servos
    final double LFTGATECLOSE_POSITION = 0.3;

    final double RRBOOTOPEN_POSITION = 0.75; //the open and close position for the rear boot servo
    final double RRBOOTCLOSE_POSITION = 0.0;

    final double LEFT_POSITION = 0.3; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0.0;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private DcMotor leftElevator = null;
    private DcMotor rightElevator = null;
    private CRServo frontFeeder = null;
    private Servo rearBoot = null;
    private Servo diverter = null;
    private Servo leftGate = null;
    private Servo rightGate = null;

    //Declare timers for Boot Close and Open
    ElapsedTime bootCloseTimer = new ElapsedTime();
    ElapsedTime bootOpenTimer = new ElapsedTime();
    ElapsedTime sleepTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        BOOT,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum LeftGateState {
        LFTGATEOPEN,
        LFTGATECLOSE;
    }
    private LeftGateState leftGateState = LeftGateState.LFTGATECLOSE;

    private enum RightGateState {
        RTGATEOPEN,
        RTGATECLOSE;
    }
    private RightGateState rightGateState = RightGateState.RTGATECLOSE;

    private enum RearBootState {
        RRBOOTOPEN,
        RRBOOTCLOSE;
    }
    private RearBootState rearBootState = RearBootState.RRBOOTOPEN;

    private enum IntakeState {
        ON,
        OFF;
    }

    private enum ElevatorState {
        HIGH,
        LOW;
    }

    private IntakeState intakeState = IntakeState.OFF;

    private ElevatorState elevatorState = ElevatorState.LOW;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Camera Code Initializations

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Camera Code Initializations

        // Initialize the Apriltag Detection process
        initAprilTag();

        //if (USE_WEBCAM)
        //    setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");

        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftElevator = hardwareMap.get(DcMotorEx.class, "leftElevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rightElevator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rearBoot = hardwareMap.get(Servo.class, "rearFeeder");
        frontFeeder = hardwareMap.get(CRServo.class, "frontFeeder");
        diverter = hardwareMap.get(Servo.class, "diverter");
        leftGate = hardwareMap.get(Servo.class, "leftGate");
        rightGate = hardwareMap.get(Servo.class, "rightGate");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        leftElevator.setZeroPowerBehavior(BRAKE);
        rightElevator.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Set Feeders to an initial value to initialize the servo controller
         */
        //rearFeeder.setPower(STOP_SPEED);
        frontFeeder.setPower(STOP_SPEED);
        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the front feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        //rearFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * set gates and boot
         */
        rearBoot.setPosition(RRBOOTOPEN_POSITION);
        rightGate.setPosition(RTGATECLOSE_POSITION);
        leftGate.setPosition(LFTGATECLOSE_POSITION);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        //Camera Code
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        //targetFound = false;
        //private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
        desiredTag  = null;


        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Driver left bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }

        // Apply desired axes motions to the drivetrain.
        mecanumDrive(drive, -strafe, -turn);
        //sleep(10);
        sleepTimer.reset();
        if (sleepTimer.seconds() > 10) {}

        //Camera Code


        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the power to elevator motors for the end game.
         */
        if (gamepad1.yWasPressed()){
            switch (elevatorState){
                case LOW:
                    elevatorState = ElevatorState.HIGH;
                    leftElevator.setPower(1);
                    rightElevator.setPower(1);
                    break;
                case HIGH:
                    elevatorState = ElevatorState.LOW;
                    leftElevator.setPower(0);
                    rightElevator.setPower(0);
                    break;
            }
        } else if (gamepad1.b) { // Bring down elevator
            elevatorState = ElevatorState.LOW;
            leftElevator.setPower(-1);
            rightElevator.setPower(-1);
        } else if (gamepad1.x) { // Powerdown elevator
            leftElevator.setPower(0);
            rightElevator.setPower(0);
        } else if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(1);
                    break;
            }
        }

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    break;
            }
        }

        /*
         * Here we give the user control to start the launcher, intake and front feeder with Y
         * and stop the launcher and front feeder with B
         */
        if (gamepad2.y) {
            launcher.setVelocity(launcherTarget);
            frontFeeder.setPower(FULL_SPEED);
            intakeState = IntakeState.ON;
            intake.setPower(1);
        } else if (gamepad2.b) { // stop flywheel and front feeder
            launcher.setVelocity(STOP_SPEED);
            frontFeeder.setPower(STOP_SPEED);
            intakeState = IntakeState.OFF;
            intake.setPower(0);
        }


        /*
         * Here we give the user control to guide the intake to left or right
         */
        if (gamepad2.dpadDownWasPressed()) {
            switch (diverterDirection){
                case LEFT:
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    break;
                case RIGHT:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    break;
            }
        }

        /*
         * Here we give the user control to start the intake
         */
        if (gamepad2.aWasPressed()){
            /* switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(1);
                    break;
            } */
            switch (rightGateState){
                case RTGATEOPEN:
                    rightGateState = RightGateState.RTGATECLOSE;
                    rightGate.setPosition(RTGATECLOSE_POSITION);
                    break;
                case RTGATECLOSE:
                    rightGateState = RightGateState.RTGATEOPEN;
                    rightGate.setPosition(RTGATEOPEN_POSITION);
                    break;
            }
        }


        /*
         * Here we give the user control to open close left gate
         */
        if (gamepad2.xWasPressed()){
            switch (leftGateState){
                case LFTGATEOPEN:
                    leftGateState = LeftGateState.LFTGATECLOSE;
                    leftGate.setPosition(LFTGATECLOSE_POSITION);
                    break;
                case LFTGATECLOSE:
                    leftGateState = LeftGateState.LFTGATEOPEN;
                    leftGate.setPosition(LFTGATEOPEN_POSITION);
                    break;
            }
        }

        /*
         * Now we call our "Launch" function.
         */

        launchLeft(gamepad2.leftBumperWasPressed());
        launchRight(gamepad2.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", leftLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        //telemetry.addData("Right Launcher Velocity", rightLauncher.getVelocity());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }


    //Shooting with the left gate open
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                //launcher.setVelocity(launcherTarget);

                //if (launcher.getVelocity() > launcherMin) {
                //frontFeeder.setPower(FULL_SPEED);
                leftLaunchState = LaunchState.LAUNCH;
                //}
                break;
            case LAUNCH:

                leftGateState = LeftGateState.LFTGATEOPEN;
                leftGate.setPosition(LFTGATEOPEN_POSITION);
                bootCloseTimer.reset();

                leftLaunchState = LaunchState.BOOT;
                break;
            case BOOT:

                if(bootCloseTimer.seconds() > BOOTCLOSE_TIME_SECONDS){
                    leftGateState = LeftGateState.LFTGATECLOSE;
                    leftGate.setPosition(LFTGATECLOSE_POSITION);
                    rearBootState = RearBootState.RRBOOTCLOSE;
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    leftLaunchState = LaunchState.LAUNCHING;
                }
                bootOpenTimer.reset();
                break;
            case LAUNCHING:

                if (bootOpenTimer.seconds() > BOOTOPEN_TIME_SECONDS) {
                    rearBootState = RearBootState.RRBOOTOPEN;
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);

                    //frontFeeder.setPower(STOP_SPEED);
                    //launcher.setVelocity(STOP_SPEED);
                    leftLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }


    // Shooting with the right gate open
    //
    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                //launcher.setVelocity(launcherTarget);

                //if (launcher.getVelocity() > launcherMin) {
                //    frontFeeder.setPower(FULL_SPEED);
                rightLaunchState = LaunchState.LAUNCH;
                //}
                break;
            case LAUNCH:
                rightGateState = RightGateState.RTGATEOPEN;
                rightGate.setPosition(RTGATEOPEN_POSITION);
                bootCloseTimer.reset();

                rightLaunchState = LaunchState.BOOT;
                break;
            case BOOT:
                if(bootCloseTimer.seconds() > BOOTCLOSE_TIME_SECONDS){
                    rightGateState = RightGateState.RTGATECLOSE;
                    rightGate.setPosition(RTGATECLOSE_POSITION);
                    rearBootState = RearBootState.RRBOOTCLOSE;
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    rightLaunchState = LaunchState.LAUNCHING;
                }
                bootOpenTimer.reset();
                break;
            case LAUNCHING:
                if (bootOpenTimer.seconds() > BOOTOPEN_TIME_SECONDS) {
                    rearBootState = RearBootState.RRBOOTOPEN;
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    //frontFeeder.setPower(STOP_SPEED);
                    //launcher.setVelocity(STOP_SPEED);
                    rightLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    /*public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }*/


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            //telemetry.update();
            if ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleepTimer.reset();
                if (sleepTimer.seconds() > 20) {}
                //sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        //if (!isStopRequested())
        //{
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleepTimer.reset();
            if (sleepTimer.seconds() > 50) {}
            //sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleepTimer.reset();
        if (sleepTimer.seconds() > 20) {}
        //sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleepTimer.reset();
        if (sleepTimer.seconds() > 20) {}
        //sleep(20);
        //}
    }
}
