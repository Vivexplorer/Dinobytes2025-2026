package org.firstinspires.ftc.teamcode.Subsystems;

import java.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    public static DcMotorEx shooter;

    static HardwareMap hardwareMap;

    Timer shootingBallTimer;

    public static void initShooterClass() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }

    public static void ShootBall() {

    }
}
