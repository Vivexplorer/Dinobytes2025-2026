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

import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RearRedAuton")
public class RearRedAuton extends OpMode {
    Intake intake;

    Outtake outtake;

    CameraSubsystem cameraSubsystem;



    DcMotorEx launcher;

    Servo leftGate, rightGate, rearFeeder, frontFeeder;
     private Follower follower;
     private Timer pathTimer, actionTimer, opmodeTimer, shootingTimer;

     private int pathState;

    private final Pose startPose = new Pose(97, 9, Math.toRadians(90));
     private final Pose shootLong = new Pose(85.5, 15, Math.toRadians(69.5));

     private final Pose readyToIntake = new Pose(101, 32, Math.toRadians(0));

    private final Pose moveToIntake = new Pose(106, 32, Math.toRadians(0));

    private final Pose moveToPurpleIntake = new Pose(106, 36, Math.toRadians(0));

    private final Pose intakeBothPurple = new Pose(122, 36, Math.toRadians(0));
    private final Pose readyToIntake2 = new Pose(100, 56, Math.toRadians(0));

    private final Pose moveToIntake2 = new Pose(120.5,56, Math.toRadians(0));



     private final Pose moveToCloseScore = new Pose(85, 84, Math.toRadians(45));

     int aprilTagID = -1;



     private Path scorePreload;

     private Path readyToIntakeFirst;


     private Path moveToIntakeFirst;

     private Path moveToIntakePurple;

     private Path moveToIntakeBothPurple;



     public void autonomousPathUpdate() {
         switch(pathState) {
             //score the preload
             case 0:
                 cameraSubsystem.cameraLoop();
                 aprilTagID = cameraSubsystem.cameraLoop();
                 follower.setMaxPower(0.1);
                 follower.followPath(scorePreload);
                 outtake.ShootBallLoop();

                 if (aprilTagID == 21) {
                     setPathState(1);
                 } else if (aprilTagID == 22) {
                     setPathState(15);
                 } else if (aprilTagID == 23) {
                     setPathState(29);
                 } else {
                     setPathState(0);
                 }
                 break;

                 //start shooting sequence

             case 1:
                 if(pathTimer.getElapsedTimeSeconds()>0) {
                     follower.setMaxPower(1);
                     intake.spinIntake();
                     outtake.runFeeder();
                     setPathState(100);
                 }
                 break;

             case 100:
                 if (launcher.getVelocity()>1500) {
                     outtake.openLeftGate();
                     setPathState(2);
                 }
                 break;

             case 2:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeLeftGate();
                     setPathState(3);
                 }
                 break;

             case 3:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeBoot();
                     setPathState(4);
                 }
                 break;

             case 4:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.openBoot();
                     setPathState(5);
                 }
                 break;

             case 5:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.openRightGate();
                     setPathState(6);
                 }
                 break;

             case 6:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeRightGate();
                     setPathState(7);
                 }
                 break;

             case 7:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeBoot();
                     setPathState(1000);
                 }
                 break;

             case 1000:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.openBoot();
                     setPathState(8);
                 }
                 break;

             case 8:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.openGates();
                     setPathState(9);
                 }
                 break;

             case 9:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeGates();
                     setPathState(10);
                 }
                 break;

             case 10:
                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
                     outtake.closeBoot();
                     setPathState(11);
                 }

             case 11:
                 if(pathTimer.getElapsedTimeSeconds()>0.8) {
                     outtake.openBoot();
                     setPathState(50);
                 }





                 //if it is the case where motif is at 22





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

             case 27:
                 if(pathTimer.getElapsedTimeSeconds()>0.8) {
                     outtake.openBoot();
                     setPathState(50);
                 }

                 //23 shooting pattern

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
                     outtake.openRightGate();
                     setPathState(31);
                 }
                 break;

             case 31:
                 if (pathTimer.getElapsedTimeSeconds()>1.2) {
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
                     outtake.closeGates();
                     outtake.openBoot();
                     setPathState(50);
                 }
                 break;

                 //start intaking sequence

             case 50:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     follower.setMaxPower(1);
                     follower.followPath(readyToIntakeFirst);
                     setPathState(51);
                 }
                 break;

             case 51:
                 if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5) {
                     follower.setMaxPower(0.3);
                     follower.followPath(moveToIntakeFirst);
                     setPathState(52);
                 }
                 break;

             case 52:
                 if (pathTimer.getElapsedTimeSeconds()>1.5) {
                     follower.setMaxPower(0.3);
                     follower.followPath(moveToIntakePurple);
                     setPathState(53);
                 }

             case 53:
                 if (pathTimer.getElapsedTimeSeconds()>1) {
                     follower.setMaxPower(0.3);
                     follower.followPath(moveToIntakeBothPurple);
                     setPathState(54);
                 }








//                 //shoot first ball
//
//             case 2:
//                 if (pathTimer.getElapsedTimeSeconds()>2) {
//                     outtake.closeBoot();
//                     setPathState(3);
//                 }
//                 break;
//
//                 //start shooting sequence

//             case 3:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                    outtake.openBoot();
//                    outtake.closeLeftGate();
//                    outtake.closeRightGate();
//                    setPathState(4);
//
//                 }
//                 break;
//
//
//             case 4:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.openRightGate();
//                     outtake.openLeftGate();
//                     setPathState(5);
//                 }
//                 break;
//
//                 //shoot second ball
//
//             case 5:
//                 if (pathTimer.getElapsedTimeSeconds()>1.75) {
//                     outtake.closeBoot();
//                     setPathState(6);
//                 }
//                 break;
//
//             case 6:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.closeRightGate();
//                     outtake.closeLeftGate();
//                     outtake.openBoot();
//                     follower.followPath(readyToIntakeFirst);
//                     intake.spinIntake();
//                     setPathState(7);
//                 }
//                 break;
//
//             case 7:
//                 if (pathTimer.getElapsedTimeSeconds()>2) {
//                     follower.setMaxPower(0.3);
//                     follower.followPath(moveToIntakeFirst);
//                     setPathState(8);
//                 }
//                 break;
//
//             case 8:
//                 if(pathTimer.getElapsedTimeSeconds()>1.5) {
//                     follower.setMaxPower(1);
//                     follower.followPath(moveToScore);
//                     setPathState(9);
//                 }
//                 break;
//
//
//             case 9:
//                 if(pathTimer.getElapsedTimeSeconds()>1.5) {
//                     intake.spinIntake();
//                     outtake.runFeeder();
//                     outtake.openLeftGate();
//                     setPathState(10);
//                 }
//                 break;
//
//             case 10:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.closeBoot();
//                     setPathState(11);
//                 }
//                 break;
//
//             case 11:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.openBoot();
//                     outtake.closeLeftGate();
//                     outtake.closeRightGate();
//                     setPathState(12);
//
//                 }
//                 break;
//             case 12:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.openRightGate();
//                     outtake.openLeftGate();
//                     setPathState(13);
//                 }
//                 break;
//
//             case 13:
//                 if (pathTimer.getElapsedTimeSeconds()>2) {
//                     outtake.closeBoot();
//                     setPathState(14);
//                 }
//                 break;
//
//             case 14:
//                 if (pathTimer.getElapsedTimeSeconds()>0.75) {
//                     outtake.closeRightGate();
//                     outtake.closeLeftGate();
//                     outtake.openBoot();
//                     follower.followPath(readyToIntakeSecond);
//                     intake.spinIntake();
//                     setPathState(15);
//                 }
//                 break;
//             case 15:
//                 if (pathTimer.getElapsedTimeSeconds()>1.25) {
//                     follower.setMaxPower(0.3);
//                     follower.followPath(moveToIntakeSecond);
//                     setPathState(16);
//                 }
//                 break;
//             case 16:
//                 if (pathTimer.getElapsedTimeSeconds()>2) {
//                     follower.setMaxPower(1);
//                     follower.followPath(moveToCloseScore1);
//                     setPathState(17);
//                 }
//                 break;
//             case 17:
//                 if(pathTimer.getElapsedTimeSeconds()>2) {
//                     intake.spinIntake();
//                     outtake.runFeeder();
//                     outtake.openLeftGate();
//                     setPathState(18);
//                 }
//                 break;
//
//             case 18:
//                 if (pathTimer.getElapsedTimeSeconds()>2) {
//                     outtake.closeBoot();
//                     setPathState(19);
//                 }
//                 break;
//
//             case 19:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.openBoot();
//                     outtake.closeLeftGate();
//                     outtake.closeRightGate();
//                     setPathState(20);
//
//                 }
//                 break;
//             case 20:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.openRightGate();
//                     outtake.openLeftGate();
//                     setPathState(21);
//                 }
//                 break;
//
//             case 21:
//                 if (pathTimer.getElapsedTimeSeconds()>1) {
//                     outtake.closeBoot();
//                     setPathState(22);
//                 }
//                 break;
//
//
//                 //shoot the balls


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

         moveToIntakePurple = new Path(new BezierLine(moveToIntake, moveToPurpleIntake));
         moveToIntakePurple.setTangentHeadingInterpolation();
         // moveToIntakeFirst.setVelocityConstraint(0.1); (unnecessary?)

//         moveToScore = new Path(new BezierLine(moveToIntake, shootLong));
//         moveToScore.setLinearHeadingInterpolation(moveToIntake.getHeading(), shootLong.getHeading());
//
//         readyToIntakeSecond = new Path(new BezierLine(shootLong, readyToIntake2));
//         readyToIntakeSecond.setLinearHeadingInterpolation(shootLong.getHeading(), readyToIntake2.getHeading());
////
//         moveToIntakeSecond = new Path(new BezierLine(readyToIntake2, moveToIntake2));
//         moveToIntakeSecond.setTangentHeadingInterpolation();
//
//         moveToCloseScore1 = new Path(new BezierLine(moveToIntake2, moveToCloseScore));
//         moveToCloseScore1.setLinearHeadingInterpolation(moveToIntake2.getHeading(), moveToCloseScore.getHeading());
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
