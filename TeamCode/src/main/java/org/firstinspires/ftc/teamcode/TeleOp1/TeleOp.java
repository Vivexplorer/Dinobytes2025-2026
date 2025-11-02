package org.firstinspires.ftc.teamcode.TeleOp1;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Initital TeleOp")
public class TeleOp extends OpMode {

    // Motors used on the robot

    DcMotorEx frontLeft;

    DcMotorEx frontRight;

    DcMotorEx rearLeft;

    DcMotorEx rearRight;

    DcMotorEx launcher;

    DcMotorEx intake;

    DcMotorEx leftLift;

    DcMotorEx rightLift;

    // Servos used on the Robot

    Servo leftGate;

    Servo rightGate;

    Servo rearFeeder;

    Servo frontFeeder;

    ElapsedTime timer = new ElapsedTime();

    Outtake outtake;

    double reducedPower = 0.5;

    //Servo diverter;

    enum States {

        Intake,

        Close_Shooting,

        Far_Shooting,

        Top_Of_Triangle_Shooting,

        Parking_Lifting,

    }

    //creating elapsed time instance
    ElapsedTime timeSinceLastChange = new ElapsedTime();

    //initialize the opmode
    @Override
    public void init() {

        outtake = new Outtake(hardwareMap);

        //Drivetrain motors Hardware Map
        rearRight = hardwareMap.get(DcMotorEx.class,"leftBackDrive");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        //Driver 2 motors Hardware Map
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightElevator");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftElevator");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //Servos Hardware Map
        leftGate = hardwareMap.get(Servo.class, "leftGate");
        rightGate = hardwareMap.get(Servo.class, "rightGate");
        rearFeeder = hardwareMap.get(Servo.class, "rearFeeder");
        frontFeeder = hardwareMap.get(Servo.class, "frontFeeder");




        // Resetting and enabling the elevator encoders
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        outtake.closeGates();

    }
    @Override
    public void loop() {




        if (gamepad2.right_trigger > 0) {
            launcher.setPower(-1);
        }
        else {
            launcher.setPower(0);
        }

        if (gamepad2.left_trigger > 0) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }

        if (gamepad2.a){
            launcher.setPower(-0.8);
            //            while (outtake.launcher.getVelocity()<outtake.velocityWanted || outtake.launcher.getVelocity()>outtake.velocityWanted) {
//                outtake.launcher.setVelocity(outtake.error);
//            }
            frontFeeder.setPosition(1.0);
            rearFeeder.setPosition(0.0);
            timer.reset();
            while (timer.seconds() < 2.2) {
                // Optional: Add telemetry to monitor
                telemetry.addData("Status", "Motor accelerating...");
                telemetry.addData("Time", "%.1f seconds", timer.seconds());
                telemetry.update();
            }

            outtake.openGates();
//            frontFeeder.setPosition(1.0);
//            rearFeeder.setPosition(0.0);
            timer.reset();
            while (timer.seconds() < 0.2) {
                outtake.closeGates();
            }
            timer.reset();
            while (timer.seconds() < 0.2) {
                outtake.openGates();

            }
        }

        if (gamepad2.y){
            outtake.launcher.setPower(-0.63);
            frontFeeder.setPosition(1.0);
            rearFeeder.setPosition(0.0);
            timer.reset();
            while (timer.seconds() < 2.5) {
                // Optional: Add telemetry to monitor
                telemetry.addData("Status", "Motor accelerating...");
                telemetry.addData("Time", "%.1f seconds", timer.seconds());
                telemetry.update();
            }

            outtake.openGates();

            timer.reset();
            while (timer.seconds() < 0.2) {
                outtake.closeGates();
            }
            timer.reset();
            while (timer.seconds() < 0.3) {
                outtake.openGates();

            }
        }


        if (gamepad2.b){
            outtake.openGates();
        }
        if (gamepad2.x){
            outtake.closeGates();
            frontFeeder.setPosition(0.5);
            rearFeeder.setPosition(0.5);
        }

        if (gamepad1.a){
            frontLeft.setPower((gamepad1.left_stick_y/2) + (-gamepad1.left_stick_x/2) + (-gamepad1.right_stick_x/2));
            frontRight.setPower((-gamepad1.left_stick_y/2) + (-gamepad1.left_stick_x/2) + (-gamepad1.right_stick_x/2));
            rearLeft.setPower((-gamepad1.left_stick_y/2) + (gamepad1.left_stick_x/2) + (-gamepad1.right_stick_x/2));
            rearRight.setPower((gamepad1.left_stick_y/2) + (gamepad1.left_stick_x/2) + (-gamepad1.right_stick_x/2));
        }
        else {
            frontLeft.setPower((gamepad1.left_stick_y) + (-gamepad1.left_stick_x) + (-gamepad1.right_stick_x));
            frontRight.setPower((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x) + (-gamepad1.right_stick_x));
            rearLeft.setPower((-gamepad1.left_stick_y) + (gamepad1.left_stick_x) + (-gamepad1.right_stick_x));
            rearRight.setPower((gamepad1.left_stick_y) + (gamepad1.left_stick_x) + (-gamepad1.right_stick_x));

        }



    }

}