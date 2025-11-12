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



@Autonomous(name = "FrontRedAuton")
public class FrontRedAuton extends OpMode {
    Intake intake;

    Outtake outtake;

    CameraSubsystem cameraSubsystem;



    DcMotorEx launcher;

    Servo leftGate, rightGate, rearFeeder, frontFeeder;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

    private int pathState;

    private final Pose startPose = new Pose(120.907, 125.977, Math.toRadians(217));

    private final Pose scoreShort = new Pose(83.7, 81, Math.toRadians(90));



    private Path scanAprilTag;

    private Path moveToScore1;








    int aprilTagID = -1;









    public void autonomousPathUpdate() {
        switch(pathState) {
            //score the preload
            case -2:
                follower.followPath(scanAprilTag);
                intake.spinIntake();
                outtake.ShootBallLoop();
                setPathState(0);
                break;

            case -1:
                if (!follower.isBusy()) {
                    cameraSubsystem.cameraLoop();
                    aprilTagID = cameraSubsystem.cameraLoop();

                    if (aprilTagID == 21) {
                        setPathState(0);
                    } else if (aprilTagID == 22) {
                        setPathState(16);
                    } else if (aprilTagID == 23) {
                        setPathState(30);
                    } else {
                        setPathState(-1);
                    }
                }
                break;

            case 0:
                if (pathTimer.getElapsedTimeSeconds()>2) {
                    follower.followPath(moveToScore1);
                    setPathState(1);
                }

            case 1:
                if(pathTimer.getElapsedTimeSeconds()>0) {
                    follower.setMaxPower(1);
                    intake.spinIntake();
                    outtake.runFeeder();
                    setPathState(2);
                }
                break;

            case 2:
                if (launcher.getVelocity()>1500) {
                    Outtake.rightGate.setPosition(0.6);
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
                    setPathState(14);
                }
                break;




            //start shooting sequence







        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    public void buildPaths() {
        scanAprilTag = new Path(new BezierCurve(startPose, new Pose(100.63, 84.86), new Pose(87.3, 84.9)));
        scanAprilTag.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(90));

        moveToScore1 = new Path(new BezierLine(new Pose(87.3, 84.9), scoreShort));
        moveToScore1.setLinearHeadingInterpolation(Math.toRadians(90), scoreShort.getHeading());






 }


    @Override
    public void loop() {

        cameraSubsystem.cameraLoop();
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
