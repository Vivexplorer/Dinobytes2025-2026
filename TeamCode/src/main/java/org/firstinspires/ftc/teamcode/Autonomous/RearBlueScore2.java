package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LauncherPID;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RearBlueScore2")
public class RearBlueScore2 extends OpMode {
    Intake intake;

    Outtake outtake;






    LauncherPID launcherPID;

    Servo leftGate, rightGate, rearFeeder, frontFeeder;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

    private int pathState;

    private final Pose startPose = new Pose(60.5, 9, Math.toRadians(90));
    private final Pose shootLong = new Pose(62.5, 15, Math.toRadians(112.5));

    private final Pose readyToIntake = new Pose(47, 33, Math.toRadians(180));



    int counter = 0;






    int aprilTagID = -1;



    private Path scorePreload;

    private Path readyToIntakeFirst;

    private Path line1IntakeGreen;



    private Path line1moveToScore;

    private Path moveOutOfScore;





    public void autonomousPathUpdate() {
        switch(pathState) {
            //score the preload
            case 0:

                follower.setMaxPower(1);
                follower.followPath(scorePreload);
                launcherPID.StartShooter();

                counter++;
                setPathState(1);

                break;


            case 1:
                if(!follower.isBusy()) {

                    intake.spinIntake();
                    outtake.runFeeder();
                    setPathState(3);
                }
                break;

//            case 100:
//                if (launcherPID.launcher.getVelocity()<-1450) {
//                    setPathState(2);
//                }
//                break;
//
//            case 2:
//                if (pathTimer.getElapsedTimeSeconds()>0.5) {
//                    setPathState(3);
//                }
//                break;

            case 3:
                if (launcherPID.launcher.getVelocity()<-1530) {
                    outtake.closeBoot();
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(7);


                }
                break;

//            case 5:
//                if (pathTimer.getElapsedTimeSeconds()>0.75) {
//                    outtake.openRightGate();
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (pathTimer.getElapsedTimeSeconds()>1.25) {
//                    outtake.closeRightGate();
//                    setPathState(7);
//                }
//                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds()>0.60) {
                    outtake.closeBoot();
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>0.7) {
                    outtake.openBoot();
                    setPathState(9);


                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    outtake.closeBoot();
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    outtake.closeBoot();
                    setPathState(12);
                }
                break;

            case 12:
                if(pathTimer.getElapsedTimeSeconds()>0.60) {
                    outtake.openBoot();
                    if (counter == 2) {
                        setPathState(1000000);
                    } else {
                        setPathState(50);
                    }
                }
                break;



            //start intaking



            case 50:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    launcherPID.StopShooter();
                    Outtake.leftGate.setPosition(0.35);
                    Outtake.rightGate.setPosition(0.7);
                    intake.intake.setPower(1);
                    follower.setMaxPower(1);
                    follower.followPath(readyToIntakeFirst);
                    setPathState(51);
                }
                break;

            case 51:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    LauncherPID.velocity = 0;
                    follower.setMaxPower(0.28);
                    follower.followPath(line1IntakeGreen);
                    setPathState(55);
                }
                break;



            case 55:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(line1moveToScore);
                    setPathState(200);
                }
                break;

            case 200:
                if (!follower.isBusy()) {
                    launcherPID.StartShooter();
                    setPathState(56);
                }
                break;



            case 56:


                if (launcherPID.launcher.getVelocity()<-1530) {

                    counter = 2;
                    setPathState(1);



                }
                break;


            case 1000000:
                if (pathTimer.getElapsedTimeSeconds()>0.4) {
                    follower.followPath(moveOutOfScore);
                    setPathState(1000001);
                }
                break;










        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, shootLong));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), shootLong.getHeading());

        readyToIntakeFirst = new Path(new BezierLine(shootLong, readyToIntake));
        readyToIntakeFirst.setLinearHeadingInterpolation(shootLong.getHeading(), readyToIntake.getHeading());

        line1IntakeGreen = new Path(new BezierLine(readyToIntake, new Pose(14,33)));
        line1IntakeGreen.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));



        line1moveToScore = new Path(new BezierCurve(new Pose(14,33), new Pose(37, 26.5), shootLong));
        line1moveToScore.setLinearHeadingInterpolation(Math.toRadians(180), shootLong.getHeading() + Math.toRadians(3));

        moveOutOfScore = new Path(new BezierLine(shootLong, new Pose(shootLong.getX()-5, shootLong.getY()+8)));
        moveOutOfScore.setLinearHeadingInterpolation(shootLong.getHeading() + Math.toRadians(3), Math.toRadians(90));






    }


    @Override
    public void loop() {
        launcherPID.shooterLoop();


        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.update();

    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootingTimer = new Timer();
        opmodeTimer.resetTimer();

        outtake = new Outtake(hardwareMap);
        launcherPID = new LauncherPID(hardwareMap);
        intake = new Intake(hardwareMap);






        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);




        leftGate = hardwareMap.get(Servo.class, "leftGate");
        rightGate = hardwareMap.get(Servo.class, "rightGate");

        outtake.closeGates();
        outtake.openBoot();

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

    }

    @Override
    public void stop() {

    }
}

