package org.firstinspires.ftc.teamcode.OldProgram;



import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the DinoByte! goBILDA® Robot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@Disabled

@TeleOp(name = "DecodeDinobyte", group = "StarterBot")
//@Disabled
public class DecodeDinobyte extends OpMode {
    final double FEED_TIME_SECONDS = 4.0; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1700; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1675; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 2025; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 2000; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.3; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    final double RTGATEOPEN_POSITION = 0.3; //the open and close position for the gate servos
    final double RTGATECLOSE_POSITION = 0.8;

    final double LFTGATEOPEN_POSITION = 0.9; //the open and close position for the gate servos
    final double LFTGATECLOSE_POSITION = 0.2;

    final double RRBOOTOPEN_POSITION = 1.4; //the open and close position for the rear boot servo
    final double RRBOOTCLOSE_POSITION = -0.10;

    // Declare OpMode members.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    private DcMotorEx launcher = null;
    //private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private DcMotor leftElevator = null;
    private DcMotor rightElevator = null;
    private Servo frontFeeder = null;
    private Servo rearBoot = null;
    private Servo diverter = null;
    private Servo leftGate = null;
    private Servo rightGate = null;

    ElapsedTime rearFeederTimer = new ElapsedTime();
    ElapsedTime frontFeederTimer = new ElapsedTime();
    ElapsedTime frontFeederTimer2 = new ElapsedTime();
    ElapsedTime buttonTimer = new ElapsedTime();


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
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rearLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rearRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        //leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftElevator = hardwareMap.get(DcMotorEx.class, "leftElevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rightElevator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rearBoot = hardwareMap.get(Servo.class, "rearFeeder");
        frontFeeder = hardwareMap.get(Servo.class, "frontFeeder");
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
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        rearLeft.setZeroPowerBehavior(BRAKE);
        rearRight.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        //rightLauncher.setZeroPowerBehavior(BRAKE);
        leftElevator.setZeroPowerBehavior(BRAKE);
        rightElevator.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        //rearFeeder.setPower(STOP_SPEED);1m

        //leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        //rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the front feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        //rearFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        rearBoot.setPosition(RRBOOTOPEN_POSITION);
        rightGate.setPosition(RTGATECLOSE_POSITION);
        leftGate.setPosition(LFTGATECLOSE_POSITION);

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

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Here we give the user control of the power to elevator motors for the end game.
         */
        if (gamepad1.yWasPressed()){
            switch (elevatorState){
                case HIGH:
                    elevatorState = ElevatorState.LOW;
                    leftElevator.setPower(0);
                    rightElevator.setPower(0);
                    break;
                case LOW:
                    elevatorState = ElevatorState.HIGH;
                    leftElevator.setPower(1);
                    rightElevator.setPower(1);
                    break;
            }
        } else if (gamepad1.b) { // Bring down elevator
            switch (elevatorState){
                case HIGH:
                    elevatorState = ElevatorState.LOW;
                    leftElevator.setPower(-1);
                    rightElevator.setPower(-1);
                    break;
                case LOW:
                    elevatorState = ElevatorState.HIGH;
                    leftElevator.setPower(0);
                    rightElevator.setPower(0);
                    break;
            }
        }

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad2.y) {
            launcher.setVelocity(launcherTarget);
            //rightLauncher.setVelocity(launcherTarget);
        } else if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
            //rightLauncher.setVelocity(STOP_SPEED);
        }

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

        if (gamepad2.aWasPressed()){
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

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        rearLeft.setPower(leftBackPower);
        rearRight.setPower(rightBackPower);

    }
    //Shooting with the left gate open
    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTarget);

                if (launcher.getVelocity() > launcherMin) {
                    frontFeeder.setPosition(FULL_SPEED);
                    frontFeederTimer.reset();
                    rightLaunchState = LaunchState.LAUNCH;

                }
                break;
            case LAUNCH:

                rightGateState = RightGateState.RTGATEOPEN;
                rightGate.setPosition(RTGATEOPEN_POSITION);
                buttonTimer.reset();

                rightLaunchState = LaunchState.BOOT;
                break;
            case BOOT:

                if(buttonTimer.seconds() > 1.0){
                    rearBootState = RearBootState.RRBOOTCLOSE;
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    rightLaunchState = LaunchState.LAUNCHING;
                }

                break;
            case LAUNCHING:

                if (frontFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rearBootState = RearBootState.RRBOOTOPEN;
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    rightGateState = RightGateState.RTGATECLOSE;
                    rightGate.setPosition(RTGATECLOSE_POSITION);
                    frontFeeder.setPosition(STOP_SPEED);
                    rightLaunchState = LaunchState.IDLE;
                    launcher.setVelocity(STOP_SPEED);
                }
                break;
        }
    }
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(launcherTarget);

                if (launcher.getVelocity() > launcherMin) {
                    frontFeeder.setPosition(FULL_SPEED);
                    frontFeederTimer.reset();
                    leftLaunchState = LaunchState.LAUNCH;

                }
                break;
            case LAUNCH:

                leftGateState = LeftGateState.LFTGATEOPEN;
                leftGate.setPosition(LFTGATEOPEN_POSITION);
                buttonTimer.reset();

                leftLaunchState = LaunchState.BOOT;
                break;
            case BOOT:

                if(buttonTimer.seconds() > 1.0){
                    rearBootState = RearBootState.RRBOOTCLOSE;
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    leftLaunchState = LaunchState.LAUNCHING;
                }

                break;
            case LAUNCHING:

                if (frontFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rearBootState = RearBootState.RRBOOTOPEN;
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    leftGateState = LeftGateState.LFTGATECLOSE;
                    leftGate.setPosition(LFTGATECLOSE_POSITION);
                    frontFeeder.setPosition(STOP_SPEED);
                    leftLaunchState = LaunchState.IDLE;
                    launcher.setVelocity(STOP_SPEED);
                }
                break;
            // Shooting with the right gate open
//    void launchRight(boolean shotRequested) {
//        switch (rightLaunchState) {
//            case IDLE:
//                if (shotRequested) {
//                    rightLaunchState = LaunchState.SPIN_UP;
//                }
//                break;
//            case SPIN_UP:
//                //leftLauncher.setVelocity(launcherTarget);
//                launcher.setVelocity(launcherTarget);
//                if (launcher.getVelocity() > launcherMin) {
//                    frontFeeder.setPower(FULL_SPEED);
//                    frontFeederTimer.reset();
//                    rightLaunchState = LaunchState.LAUNCH;
//                }
//                break;
//            case LAUNCH:
//                // frontFeeder.setPower(FULL_SPEED);
//                // frontFeederTimer.reset();
//
//                rightGateState = RightGateState.RTGATEOPEN;
//                rightGate.setPosition(RTGATEOPEN_POSITION);
//                rightLaunchState = LaunchState.BOOT;
//                buttonTimer.reset();
//                break;
//
//            case BOOT:
//                if(buttonTimer.seconds() > 1.5){
//                    rearBootState = RearBootState.RRBOOTCLOSE;
//                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
//                    leftLaunchState = LaunchState.LAUNCHING;
//
//
//                }
//                break;
//
//            case LAUNCHING:
//                if (frontFeederTimer.seconds() > FEED_TIME_SECONDS) {
//                    rearBootState = RearBootState.RRBOOTOPEN;
//                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
//                    frontFeeder.setPower(STOP_SPEED);
//                    rightGateState = RightGateState.RTGATECLOSE;
//                    rightGate.setPosition(RTGATECLOSE_POSITION);
//                    rightLaunchState = LaunchState.IDLE;
//                    launcher.setVelocity(STOP_SPEED);
//                }
//
//                //if (rearFeederTimer.seconds() > FEED_TIME_SECONDS) {
//                //    rearFeeder.setPower(STOP_SPEED);
//                //    rightLaunchState = LaunchState.IDLE;
//                //    launcher.setVelocity(STOP_SPEED);
//                //}
////                rearBoot.setPosition(RRBOOTOPEN_POSITION);
//                break;

        }
    }
}