package org.firstinspires.ftc.teamcode.TeleOp1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    HardwareMap hardwareMap;

    public DcMotorEx intake;

    public Intake(HardwareMap hardwareMap) {

        this.hardwareMap= hardwareMap;
        intake = hardwareMap.get(DcMotorEx.class, "intake");

    }

    public void spinIntake() {
        intake.setPower(1);
    }

    public void reverseIntake() {intake.setPower(-1);}

}
