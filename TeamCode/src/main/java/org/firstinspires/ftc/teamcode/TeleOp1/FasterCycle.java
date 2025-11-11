/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.TeleOp1;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FasterCycle", group = "StarterBot")
public class FasterCycle extends OpMode {

    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1670;
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1645;

    final double LAUNCHER_FAR_TARGET_VELOCITY = 2025;
    final double LAUNCHER_FAR_MIN_VELOCITY = 2000;

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double RTGATEOPEN_POSITION = 0.15;
    final double RTGATECLOSE_POSITION = 0.72;
    final double LFTGATEOPEN_POSITION = 0.45;
    final double LFTGATECLOSE_POSITION = 0.15;
    final double RRBOOTOPEN_POSITION = 1.4;
    final double RRBOOTCLOSE_POSITION = -0.10;
    private boolean intakeOn = false;

    private DcMotor frontLeft, frontRight, rearLeft, rearRight, intake;
    private DcMotorEx launcher;
    private DcMotorEx leftElevator, rightElevator;
    private CRServo frontFeeder;
    private Servo rearBoot, diverter, leftGate, rightGate;

    Outtake outtake1;

    ElapsedTime frontFeederTimer = new ElapsedTime();
    ElapsedTime buttonTimer = new ElapsedTime();
    ElapsedTime thirdBoot = new ElapsedTime();
    ElapsedTime secondBootBack = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LEFT_LAUNCH,
        LEFT_BOOT,
        LEFT_DONE,
        RIGHT_LAUNCH,
        RIGHT_BOOT,
        RIGHT_DONE,
        THIRDBOOT_LAUNCH,
        THRIDBOOT_DONE,
        STOP
    }

    private LaunchState launchState = LaunchState.IDLE;

    private enum LauncherDistance {
        CLOSE,
        FAR
    }
    private enum ElevatorState {
        HIGH,
        LOW;
    }
    private DecodeDinobyte.ElevatorState elevatorState = DecodeDinobyte.ElevatorState.LOW;

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    @Override
    public void init() {
        Outtake Outtake = new Outtake(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rearLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rearRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftElevator = hardwareMap.get(DcMotorEx.class, "leftElevator");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rightElevator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rearBoot = hardwareMap.get(Servo.class, "rearFeeder");
        diverter = hardwareMap.get(Servo.class, "diverter");
        leftGate = hardwareMap.get(Servo.class, "leftGate");
        rightGate = hardwareMap.get(Servo.class, "rightGate");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        rearLeft.setZeroPowerBehavior(BRAKE);
        rearRight.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        leftElevator.setZeroPowerBehavior(BRAKE);
        rightElevator.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearBoot.setPosition(RRBOOTOPEN_POSITION);
        rightGate.setPosition(RTGATECLOSE_POSITION);
        leftGate.setPosition(LFTGATECLOSE_POSITION);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Drive controls
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // Toggle launcher distance
        if (gamepad2.dpadUpWasPressed()) {
            if (launcherDistance == LauncherDistance.CLOSE) {
                launcherDistance = LauncherDistance.FAR;
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
            } else {
                launcherDistance = LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
            }
        }

        // BACK button stops everything immediately
        if (gamepad2.back) {
            launcherTarget = 0;
            launcher.setVelocity(0);
            leftGate.setPosition(LFTGATECLOSE_POSITION);
            rightGate.setPosition(RTGATECLOSE_POSITION);
            rearBoot.setPosition(RRBOOTOPEN_POSITION);
            launchState = LaunchState.IDLE;   // reset state
            //outtake1.stopFeeder();            // stop feeder in Outtake1
            // stop front feeder CRServo
            buttonTimer.reset();
            frontFeederTimer.reset();
        }

        if (gamepad1.yWasPressed()){
            switch (elevatorState){
                case HIGH:
                    elevatorState = DecodeDinobyte.ElevatorState.LOW;
                    leftElevator.setPower(0);
                    rightElevator.setPower(0);
                    break;
                case LOW:
                    elevatorState = DecodeDinobyte.ElevatorState.HIGH;
                    leftElevator.setPower(1);
                    rightElevator.setPower(1);
                    break;
            }
        } else if (gamepad1.b) { // Bring down elevator
            switch (elevatorState){
                case HIGH:
                    elevatorState = DecodeDinobyte.ElevatorState.LOW;
                    leftElevator.setPower(-1);
                    rightElevator.setPower(-1);
                    break;
                case LOW:
                    elevatorState = DecodeDinobyte.ElevatorState.HIGH;
                    leftElevator.setPower(0);
                    rightElevator.setPower(0);
                    break;
            }
        }
        // Intake toggle with 'A'
        if (gamepad2.aWasPressed()) {
            intakeOn = !intakeOn;
        }
        intake.setPower(intakeOn ? 1.0 : 0.0);

        // Right trigger starts launch sequence only if IDLE
        if (gamepad2.right_trigger > 0.5 && launchState == LaunchState.IDLE) {
            // Set launcher target back to current distance
            if (launcherDistance == LauncherDistance.CLOSE) {
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
            } else {
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
            }
            launchState = LaunchState.SPIN_UP;
        }

        // Run the launch sequence
        launchSequence();

        // Telemetry
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.addData("Launch State", launchState);
    }



    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double lf = (forward + strafe + rotate) / denominator;
        double rf = (forward - strafe - rotate) / denominator;
        double lb = (forward - strafe + rotate) / denominator;
        double rb = (forward + strafe - rotate) / denominator;
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        rearLeft.setPower(lb);
        rearRight.setPower(rb);
    }

    void launchSequence() {
        switch (launchState) {
            case SPIN_UP:
                launcher.setVelocity(launcherTarget);
                if (launcher.getVelocity() > launcherMin) {
                    Outtake.runFeeder();
                    frontFeederTimer.reset();
                    launchState = LaunchState.LEFT_LAUNCH;
                }
                break;

            case LEFT_LAUNCH:
                leftGate.setPosition(LFTGATEOPEN_POSITION);
                buttonTimer.reset();
                launchState = LaunchState.LEFT_BOOT;
                break;

            case LEFT_BOOT:
                if (buttonTimer.seconds() > 2.0) {
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    launchState = LaunchState.LEFT_DONE;
                    frontFeederTimer.reset();
                }
                break;

            case LEFT_DONE:
                if (frontFeederTimer.seconds() > 1.0) {
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    leftGate.setPosition(LFTGATECLOSE_POSITION);
                    launchState = LaunchState.RIGHT_LAUNCH;
                    buttonTimer.reset();
                }
                break;

            case RIGHT_LAUNCH:
                rightGate.setPosition(RTGATEOPEN_POSITION);
                leftGate.setPosition(LFTGATEOPEN_POSITION);
                buttonTimer.reset();
                launchState = LaunchState.RIGHT_BOOT;
                break;

            case RIGHT_BOOT:
                if (buttonTimer.seconds() > 1.75) {
                    rearBoot.setPosition(RRBOOTCLOSE_POSITION);
                    leftGate.setPosition(LFTGATECLOSE_POSITION);
                    launchState = LaunchState.RIGHT_DONE;
                    frontFeederTimer.reset();
                }
                break;

            case RIGHT_DONE:
                if (frontFeederTimer.seconds() > 1.0) {
                    rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    rightGate.setPosition(RTGATECLOSE_POSITION);
                    leftGate.setPosition(LFTGATEOPEN_POSITION);
                    rightGate.setPosition(RTGATEOPEN_POSITION);
                    //rearBoot.setPosition(RRBOOTOPEN_POSITION);
                    thirdBoot.reset();
                    launchState = LaunchState.THIRDBOOT_LAUNCH;
                    //outtake1.stopFeeder();     // stop feeder at end of cycle

                }
                break;
            case THIRDBOOT_LAUNCH:
                if(thirdBoot.seconds() > 1.5) {
                    rightGate.setPosition(RTGATECLOSE_POSITION);
                    leftGate.setPosition(LFTGATEOPEN_POSITION);
                    buttonTimer.reset();
                    launchState = LaunchState.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }
}
