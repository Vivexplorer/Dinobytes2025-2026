package org.firstinspires.ftc.teamcode.TeleOp1;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    public Motor frontRight;
    public Motor frontLeft;
    public Motor rearRight;
    public Motor rearLeft;

    MecanumDrive drive;

    GamepadEx driverOp;

    public Boolean fastMode = true;


    HardwareMap hardwareMap;
    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        rearRight = hardwareMap.get(Motor.class,"leftBackDrive");
        rearLeft = hardwareMap.get(Motor.class, "rightBackDrive");
        frontLeft = hardwareMap.get(Motor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(Motor.class, "rightFrontDrive");

        drive = new MecanumDrive(frontLeft, frontRight, rearLeft, rearRight);

    }

    public void DrivetrainTeleOp() {
        if (fastMode) {
            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightY()
            );
        } else {
            drive.driveRobotCentric(
                    driverOp.getLeftX()/2,
                    driverOp.getLeftY()/2,
                    driverOp.getRightY()/2
            );
        }

    }

}
