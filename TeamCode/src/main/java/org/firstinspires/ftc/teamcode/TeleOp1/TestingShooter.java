package org.firstinspires.ftc.teamcode.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

@TeleOp
public class TestingShooter extends OpMode {

    Shooter shooter;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);


    }

    @Override
    public void loop() {
        shooter.ShootBallLoop();

    }
}
