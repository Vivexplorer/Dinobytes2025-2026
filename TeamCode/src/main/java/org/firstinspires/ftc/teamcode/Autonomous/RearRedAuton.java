package org.firstinspires.ftc.teamcode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RearRedAuton")
public class RearRedAuton extends OpMode {
    Intake intake;

    Outtake outtake;



    DcMotorEx launcher;

    Servo leftGate, rightGate, rearFeeder, frontFeeder;
     private Follower follower;
     private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

     private int pathState;

    private final Pose startPose = new Pose(108, 9, Math.toRadians(90));
     private final Pose shootLong = new Pose(80, 15, Math.toRadians(65));

     private final Pose readyToIntake = new Pose(104, 37, Math.toRadians(0));

     private final Pose moveToIntake2 = new Pose(120,59, Math.toRadians(0));

     private final Pose moveToIntake = new Pose(116.5, 37, Math.toRadians(0));

     private final Pose readyToIntake2 = new Pose(102, 59, Math.toRadians(0));

     private final Pose moveToCloseScore = new Pose(85, 84, Math.toRadians(45));

     private Path scorePreload;

     private Path readyToIntakeFirst;

     private Path moveToIntakeSecond;

     private Path moveToIntakeFirst;

     private Path readyToIntakeSecond;


     private Path moveToScore;

     public void autonomousPathUpdate() {
         switch(pathState) {
             //score the preload
             case 0:
                 follower.followPath(scorePreload);
                 outtake.ShootBallLoop();
                 setPathState(1);
                 break;

             case 1:
                 if(pathTimer.getElapsedTimeSeconds()>2) {
                     intake.spinIntake();
                     outtake.runFeeder();
                     outtake.openLeftGate();
                     setPathState(2);
                 }
                 break;

             case 2:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeBoot();
                     setPathState(3);
                 }
                 break;

             case 3:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                    outtake.openBoot();
                    outtake.closeLeftGate();
                    outtake.closeRightGate();
                    setPathState(4);

                 }
                 break;
             case 4:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     outtake.openRightGate();
                     outtake.openLeftGate();
                     setPathState(5);
                 }
                 break;

             case 5:
                 if (pathTimer.getElapsedTimeSeconds()>2.5) {
                     outtake.closeBoot();
                     setPathState(6);
                 }
                 break;

             case 6:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeRightGate();
                     outtake.closeLeftGate();
                     outtake.openBoot();
                     follower.followPath(readyToIntakeFirst);
                     intake.spinIntake();
                     setPathState(7);
                 }
                 break;

             case 7:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     follower.setMaxPower(0.3);
                     follower.followPath(moveToIntakeFirst);
                     setPathState(8);
                 }
                 break;

             case 8:
                 if(pathTimer.getElapsedTimeSeconds()>2) {
                     follower.setMaxPower(1);
                     follower.followPath(moveToScore);
                     setPathState(9);
                 }
                 break;


             case 9:
                 if(pathTimer.getElapsedTimeSeconds()>2) {
                     intake.spinIntake();
                     outtake.runFeeder();
                     outtake.openLeftGate();
                     setPathState(10);
                 }
                 break;

             case 10:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeBoot();
                     setPathState(11);
                 }
                 break;

             case 11:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     outtake.openBoot();
                     outtake.closeLeftGate();
                     outtake.closeRightGate();
                     setPathState(12);

                 }
                 break;
             case 12:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     outtake.openRightGate();
                     outtake.openLeftGate();
                     setPathState(13);
                 }
                 break;

             case 13:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeBoot();
                     setPathState(14);
                 }
                 break;
             case 14:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeRightGate();
                     outtake.closeLeftGate();
                     outtake.openBoot();
                     follower.followPath(readyToIntakeSecond);
                     intake.spinIntake();
                     setPathState(15);
                 }
                 break;
             case 15:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     follower.setMaxPower(0.3);
                     follower.followPath(moveToIntakeSecond);
                     setPathState(16);
                 }
                 break;
             case 16:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     follower.setMaxPower(1);
                     follower.followPath(moveToScore);
                     setPathState(17);
                 }
                 break;
             case 17:
                 if(pathTimer.getElapsedTimeSeconds()>2) {
                     intake.spinIntake();
                     outtake.runFeeder();
                     outtake.openLeftGate();
                     setPathState(18);
                 }
                 break;

             case 18:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeBoot();
                     setPathState(19);
                 }
                 break;

             case 19:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     outtake.openBoot();
                     outtake.closeLeftGate();
                     outtake.closeRightGate();
                     setPathState(20);

                 }
                 break;
             case 20:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     outtake.openRightGate();
                     outtake.openLeftGate();
                     setPathState(21);
                 }
                 break;

             case 21:
                 if (pathTimer.getElapsedTimeSeconds()>2) {
                     outtake.closeBoot();
                     setPathState(22);
                 }


                 //shoot the balls


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

         moveToIntakeFirst = new Path(new BezierLine(readyToIntake, moveToIntake));
         moveToIntakeFirst.setTangentHeadingInterpolation();
         // moveToIntakeFirst.setVelocityConstraint(0.1); (unnecessary?)

         moveToScore = new Path(new BezierLine(moveToIntake, moveToCloseScore));
         moveToScore.setLinearHeadingInterpolation(moveToIntake.getHeading(), moveToCloseScore.getHeading());

         readyToIntakeSecond = new Path(new BezierLine(moveToCloseScore, readyToIntake2));
         readyToIntakeSecond.setLinearHeadingInterpolation(moveToCloseScore.getHeading(), readyToIntake2.getHeading());

         moveToIntakeSecond = new Path(new BezierLine(readyToIntake2, moveToIntake2));
         moveToIntakeSecond.setTangentHeadingInterpolation();
     }


     @Override
     public void loop() {
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
    public void init_loop() {}

    @Override
    public void start() {
         opmodeTimer.resetTimer();

    }

    @Override
    public void stop() {

    }
}
