package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LauncherPID;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "FrontBlueAuton6")
public class FrontBlueAuton6 extends OpMode {
    Intake intake;

    Outtake outtake;

    LauncherPID launcherPID;

    int counter;






    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

    private int pathState = -1;

    private final Pose startPose = new Pose(28, 128, Math.toRadians(143));

    private final Pose scoreShort = new Pose(60.3, 81, Math.toRadians(132));


    private final Pose readyToIntakeLine1 = new Pose(44, 80, Math.toRadians(180));

    private final Pose readyToIntakeLine2 = new Pose(48, 59, Math.toRadians(180));





    private Path moveToScore1;

    private Path line1readyToIntake;

    private Path line1moveToIntake;

    private Path moveToScore2;

    private Path line2readyToIntake;

    private Path line2moveToIntake;

    private Path openGate;

    private Path moveToScore3;

    private Path park;















    public void autonomousPathUpdate() {
        switch(pathState) {


            case -1:
                LauncherPID.velocity = 400;
                outtake.runFeeder();
                intake.spinIntake();
                follower.setMaxPower(0.6);
                follower.followPath(moveToScore1);
                counter = 1;
                setPathState(1);

                break;



            case 1:
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    intake.spinIntake();
                    LauncherPID.velocity = -1515;
                    setPathState(4);
                }
                break;

            //take out case 4 and 5



            case 4:
                if (launcherPID.launcher.getVelocity()< -1500) {
                    outtake.closeBoot();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(8);
                }
                break;





            case 8:
                if (pathTimer.getElapsedTimeSeconds()>0.9) {
                    outtake.closeBoot();
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    outtake.openBoot();
                    if (counter == 3) {
                        setPathState(200);
                    }else {
                        setPathState(12);
                    }

                }
                break;



            case 12:
                if (outtake.BallFound() || pathTimer.getElapsedTimeSeconds()> 0.75) {
                    outtake.closeBoot();
                    setPathState(13);
                }
                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds()>0.5) {
                    outtake.openBoot();
                    setPathState(6);

                }
                break;

            case 6:
                if (outtake.BallFound() || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    outtake.closeBoot();
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds()>0.6) {
                    outtake.openBoot();
                    if (counter == 1) {
                        setPathState(50);
                    } else if (counter == 2){
                        setPathState(200);
                    }
                }
                break;



            //start line 1 intake sequence

            case 50:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {

                    intake.spinIntake();
                    launcherPID.StopShooter();
                    follower.followPath(line1readyToIntake);
                    setPathState(51);
                }
                break;

            case 51:
                if (!follower.isBusy()) {
                    LauncherPID.velocity = 400;
                    follower.setMaxPower(0.5);
                    follower.followPath(line1moveToIntake);
                    setPathState(52);
                }
                break;


            case 52:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(moveToScore2);
                    counter = 2;
                    setPathState(1);
                }
                break;

            //start line 2 intake sequence

            case 53:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {

                    intake.spinIntake();
                    LauncherPID.velocity = 400;
                    follower.setMaxPower(1);
                    follower.followPath(line2readyToIntake);
                    setPathState(54);
                }
                break;

            case 54:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(line2moveToIntake);
                    setPathState(56);
                }
                break;


            //dont use opengate
            case 55:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.3);
                    follower.followPath(openGate);

                    setPathState(56);
                }
                break;

            case 56:
                if (!follower.isBusy()) {

                    setPathState(200);
                }
                break;

            //parking

            case 200:
                if (pathTimer.getElapsedTimeSeconds()>0) {
                    follower.followPath(park);
                    setPathState(201);
                }
                break;













        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    public void buildPaths() {


        moveToScore1 = new Path(new BezierLine(startPose, scoreShort));
        moveToScore1.setLinearHeadingInterpolation(startPose.getHeading(), scoreShort.getHeading());

        line1readyToIntake = new Path(new BezierLine(scoreShort, readyToIntakeLine1));
        line1readyToIntake.setLinearHeadingInterpolation(scoreShort.getHeading(), readyToIntakeLine1.getHeading());

        line1moveToIntake = new Path(new BezierLine(readyToIntakeLine1, new Pose(readyToIntakeLine1.getX()-24, 80)));
        line1moveToIntake.setLinearHeadingInterpolation(readyToIntakeLine1.getHeading(), readyToIntakeLine1.getHeading());

        moveToScore2 = new Path(new BezierLine(new Pose(readyToIntakeLine1.getX()-24, 80), scoreShort));
        moveToScore2.setLinearHeadingInterpolation(readyToIntakeLine1.getHeading(), scoreShort.getHeading());

        line2readyToIntake = new Path(new BezierLine(scoreShort, new Pose(readyToIntakeLine2.getX(), readyToIntakeLine2.getY()+3.5)));
        line2readyToIntake.setLinearHeadingInterpolation(scoreShort.getHeading(), readyToIntakeLine2.getHeading());

        line2moveToIntake = new Path(new BezierLine(new Pose(readyToIntakeLine2.getX(), readyToIntakeLine2.getY()+3.5), new Pose(21, readyToIntakeLine2.getY())));
        line2moveToIntake.setLinearHeadingInterpolation(readyToIntakeLine2.getHeading(), readyToIntakeLine2.getHeading());

        openGate = new Path(new BezierCurve(new Pose(21, readyToIntakeLine2.getY()), new Pose(22.4, 64), new Pose(17.4, 63.9)));
        openGate.setLinearHeadingInterpolation(readyToIntakeLine2.getHeading(), readyToIntakeLine2.getHeading());

        moveToScore3 = new Path(new BezierCurve(new Pose(21, readyToIntakeLine2.getY()), new Pose(42.3, 63.3), new Pose(startPose.getX()-5, startPose.getY()+20)));
        moveToScore3.setLinearHeadingInterpolation(readyToIntakeLine2.getHeading(), Math.toRadians(90));

        park = new Path(new BezierLine(scoreShort, new Pose(scoreShort.getX()-35, scoreShort.getY()+10)));
        park.setLinearHeadingInterpolation(scoreShort.getHeading(), Math.toRadians(90));







    }


    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        launcherPID.shooterLoop();

        telemetry.addData("path state", pathState);
        telemetry.addData("ball distance ", outtake.disSensor1.getDistance(DistanceUnit.MM));
        telemetry.update();

    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootingTimer = new Timer();
        opmodeTimer.resetTimer();

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        launcherPID = new LauncherPID(hardwareMap);



        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



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

