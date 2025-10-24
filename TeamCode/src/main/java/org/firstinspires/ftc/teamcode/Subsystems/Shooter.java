package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public static DcMotorEx shooter;

    static HardwareMap hardwareMap;


    public static double kP, kI, kD, kF;

    static PIDFController pidf;

    static double velocityWanted = 125.0/60;



    public static void initShooterClass() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setPIDF(kP, kI, kD, kF);

    }





    public static void ShootBallLoop() {
        while (!pidf.atSetPoint()) {

            double error = pidf.calculate(
                    shooter.getVelocity(), velocityWanted
            );

            shooter.setVelocity(error);
        }
    }
}
