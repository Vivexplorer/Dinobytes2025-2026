package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class LauncherPIDTesting extends OpMode {

    LauncherPID launcherPID;

    @Override
    public void init() {
        launcherPID = new LauncherPID(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            launcherPID.StartShooter();
        } else if (gamepad1.b) {
            launcherPID.StopShooter();
        }
        telemetry.addData("shooter current velo", launcherPID.launcher.getVelocity());
        telemetry.addData("shooter wanted velo", LauncherPID.velocity);
        telemetry.update();
        launcherPID.shooterLoop();
    }
}
