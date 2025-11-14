package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "FrontBlueAuton")
public class FrontBlueAuton extends OpMode {
    Intake intake;

    Outtake outtake;

    CameraSubsystem cameraSubsystem;



    DcMotorEx launcher;

    Servo leftGate, rightGate, rearFeeder, frontFeeder;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

    private int pathState = -2;

    private final Pose startPose = new Pose(23.093, 125.977, Math.toRadians(323));

    private final Pose scoreShort = new Pose(60.3, 78, Math.toRadians(132));



    private Path scanAprilTag;

    private Path moveToScore1;

    private Path line1readyToIntakePurple;

    private Path line1moveToIntakePurple;

    private Path line1moveAwayFromIntakePurple;

    private Path line1moveToIntakeGreen;

    private Path moveToScore2;









    int aprilTagID = -1;









    public void autonomousPathUpdate() {
        switch(pathState) {
            //get to correct scanning position
            case -2:
                follower.followPath(scanAprilTag);
                cameraSubsystem.cameraLoop();
                intake.spinIntake();
                outtake.launcher.setVelocity(1540);
                setPathState(-1);
                break;

            //scan the april tag

            case -1:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    aprilTagID = cameraSubsystem.cameraLoop();

                    if (aprilTagID == 21) {
                        follower.followPath(moveToScore1);
                        setPathState(1);
                    } else if (aprilTagID == 22) {
                        follower.followPath(moveToScore1);
                        setPathState(15);
                    } else if (aprilTagID == 23) {
                        follower.followPath(moveToScore1);
                        setPathState(29);
                    } else {
                        setPathState(-1);
                    }
                }
                break;


            //start shooting sequence for april tag id 21

            case 1:
                if(pathTimer.getElapsedTimeSeconds()>0.5) {
                    follower.setMaxPower(1);
                    intake.spinIntake();
                    outtake.runFeeder();
                    setPathState(2);
                }
                break;

            case 2:
                if (launcher.getVelocity()>1500) {
                    outtake.closeRightGate();
                    outtake.openLeftGate();
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds()>1.1) {
                    outtake.closeLeftGate();
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openRightGate();
                    setPathState(7);
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds()>1.25) {
                    outtake.closeRightGate();
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>0.60) {
                    outtake.closeBoot();
                    setPathState(9);
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds()>0.65) {
                    outtake.openGates();
                    setPathState(11);
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeGates();
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds()>0.60) {
                    outtake.closeBoot();
                    setPathState(13);
                }
                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds()>0.60) {
                    outtake.openBoot();
                    setPathState(50);
                }
                break;

            //shooting sequence for april tag id 22

            case 15:
                if(pathTimer.getElapsedTimeSeconds()>0) {
                    follower.setMaxPower(1);
                    intake.spinIntake();
                    outtake.runFeeder();
                    setPathState(16);
                }
                break;

            case 16:
                if (launcher.getVelocity()>1500) {
                    Outtake.leftGate.setPosition(0.38);
                    outtake.openRightGate();
                    setPathState(17);
                }
                break;

            case 17:
                if (pathTimer.getElapsedTimeSeconds()>1.2) {
                    outtake.closeRightGate();
                    setPathState(18);
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(20);
                }
                break;

            case 20:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openLeftGate();
                    setPathState(21);
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds()>1.2) {
                    outtake.closeLeftGate();
                    setPathState(22);
                }
                break;

            case 22:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(23);
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(24);
                }
                break;

            case 24:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openGates();
                    setPathState(25);
                }
                break;

            case 25:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeGates();
                    setPathState(26);
                }
                break;

            case 26:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(27);
                }
                break;

            case 27:
                if(pathTimer.getElapsedTimeSeconds()>0.8) {
                    outtake.openBoot();
                    setPathState(50);
                }
                break;

            //shooting sequence when tag id is 23

            case 29:
                if(pathTimer.getElapsedTimeSeconds()>0) {
                    follower.setMaxPower(1);
                    intake.spinIntake();
                    outtake.runFeeder();
                    setPathState(30);
                }
                break;

            case 30:
                if (launcher.getVelocity()>1500) {
                    Outtake.leftGate.setPosition(0.38);
                    outtake.openRightGate();
                    setPathState(31);
                }
                break;

            case 31:
                if (pathTimer.getElapsedTimeSeconds()>1.5) {
                    outtake.closeRightGate();
                    setPathState(32);
                }
                break;

            case 32:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(33);
                }
                break;

            case 33:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(34);
                }
                break;

            case 34:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openRightGate();
                    setPathState(35);
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds()>1.2) {
                    outtake.closeRightGate();
                    setPathState(36);
                }
                break;

            case 36:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(37);
                }
                break;

            case 37:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openBoot();
                    setPathState(38);
                }
                break;

            case 38:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.openGates();
                    setPathState(39);
                }
                break;

            case 39:
                if (pathTimer.getElapsedTimeSeconds()>0.0) {
                    setPathState(40);
                }
                break;

            case 40:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    outtake.closeBoot();
                    setPathState(41);
                }
                break;

            case 41:
                if(pathTimer.getElapsedTimeSeconds()>0.8) {
                    launcher.setVelocity(1000);
                    outtake.openBoot();
                    setPathState(50);
                }
                break;

            //start intaking sequence

            case 50:
                if (pathTimer.getElapsedTimeSeconds()>0.75) {
                    intake.spinIntake();
                    outtake.runFeeder();
                    follower.followPath(line1readyToIntakePurple);
                    setPathState(51);
                }
                break;

            case 51:
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    follower.setMaxPower(0.2);
                    follower.followPath(line1moveToIntakePurple);
                    setPathState(52);
                }
                break;

            case 52:
                if (pathTimer.getElapsedTimeSeconds()>2.5) {
                    follower.setMaxPower(0.3);
                    follower.followPath(line1moveAwayFromIntakePurple);
                    setPathState(53);
                }
                break;

            case 53:
                if (pathTimer.getElapsedTimeSeconds()>1.5) {
                    follower.setMaxPower(0.3);
                    follower.followPath(line1moveToIntakeGreen);
                    setPathState(54);
                }
                break;

            case 54:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(moveToScore2);
                    setPathState(55);
                }
                break;

            case 55:
                if (pathTimer.getElapsedTimeSeconds()>1) {
                    if (aprilTagID == 21) {
                        setPathState(1);
                    } else if (aprilTagID == 22) {
                        setPathState(15);
                    } else {
                        setPathState(29);
                    }
                }
                break;







        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    public void buildPaths() {
        scanAprilTag = new Path(new BezierCurve(startPose, new Pose(43.37, 84.86), new Pose(56.7, 84.9)));
        scanAprilTag.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(90));

        moveToScore1 = new Path(new BezierLine(new Pose(56.7, 78), scoreShort));
        moveToScore1.setLinearHeadingInterpolation(Math.toRadians(90), scoreShort.getHeading());

        line1readyToIntakePurple = new Path(new BezierLine(scoreShort, new Pose(44, 78)));
        line1readyToIntakePurple.setLinearHeadingInterpolation(scoreShort.getHeading(), Math.toRadians(180));

        line1moveToIntakePurple = new Path(new BezierLine(new Pose(44,78), new Pose(30, 78)));
        line1moveToIntakePurple.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        line1moveAwayFromIntakePurple = new Path(new BezierLine(new Pose(30, 78), new Pose(35, 84)));
        line1moveAwayFromIntakePurple.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        line1moveToIntakeGreen = new Path(new BezierLine(new Pose(35, 84), new Pose(24, 84)));
        line1moveToIntakeGreen.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180));

        moveToScore2 = new Path(new BezierCurve(new Pose(24, 84), new Pose(53.27, 94.2), scoreShort));
        moveToScore2.setLinearHeadingInterpolation(Math.toRadians(180), scoreShort.getHeading());

    }


    @Override
    public void loop() {

        cameraSubsystem.cameraLoop();
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("april tag id: ", aprilTagID);
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
        cameraSubsystem = new CameraSubsystem(hardwareMap);




        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

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


