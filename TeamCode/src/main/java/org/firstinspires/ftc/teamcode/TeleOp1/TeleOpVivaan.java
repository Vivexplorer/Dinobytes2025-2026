package org.firstinspires.ftc.teamcode.TeleOp1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;

@TeleOp(name = "TeleOpVivaan")
public class TeleOpVivaan extends OpMode {

    Drivetrain drivetrain;
    Intake intake;
    Outtake outtake;



    enum RobotState {
        Ready_To_Intake,

        Intaking,

        Ready_To_Outtake,

        Short_Range_Shooting,

        Long_Range_Shooting,

        Ready_To_Hang
    }

    RobotState currentRobotState = RobotState.Ready_To_Intake;

    ElapsedTime timeSinceLastChange = new ElapsedTime();

    public void setRobotState(RobotState newState) {
        currentRobotState = newState;
        timeSinceLastChange.reset();
    }


    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        outtake.closeGates();
    }

    @Override
    public void loop() {
        switch(currentRobotState) {
            case Ready_To_Intake:
                outtake.closeGates();
                break;

            case Intaking:
                intake.spinIntake();
                outtake.closeGates();
                break;

            case Ready_To_Outtake:
                outtake.ShootBallLoop();
                break;

            case Short_Range_Shooting:
                break;

            case Long_Range_Shooting:
                break;

            case Ready_To_Hang:
                break;
        }
    }
}
