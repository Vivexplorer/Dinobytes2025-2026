package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@Autonomous(name = "RearRedAuton2Ball")
public class RearRedAuton2Ball extends OpMode {

    Intake intake;

    Outtake outtake;

    Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

    private int pathState;

    private final Pose startPose = new Pose(39, 9, Math.toRadians(90));
    private final Pose shoot1Pose = new Pose(56, 15, Math.toRadians(60));

    private Path score;

    private Path getReadyforTele;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);



    }

    @Override
    public void loop() {

    }
}
