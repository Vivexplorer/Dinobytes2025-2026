package org.firstinspires.ftc.teamcode.Subsystems;



import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Shooter {
    DcMotorEx launcher;

    HardwareMap hardwareMap;

    Telemetry telemetry;







    public static double kP = 0.01, kI, kD, kF = 0.1;

    static PIDFController pidf;

    public static double velocityWanted = 125.0/60;


    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new Telemetry() {
            @Override
            public Item addData(String caption, String format, Object... args) {
                return null;
            }

            @Override
            public Item addData(String caption, Object value) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, Func<T> valueProducer) {
                return null;
            }

            @Override
            public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                return null;
            }

            @Override
            public boolean removeItem(Item item) {
                return false;
            }

            @Override
            public void clear() {

            }

            @Override
            public void clearAll() {

            }

            @Override
            public Object addAction(Runnable action) {
                return null;
            }

            @Override
            public boolean removeAction(Object token) {
                return false;
            }

            @Override
            public void speak(String text) {

            }

            @Override
            public void speak(String text, String languageCode, String countryCode) {

            }

            @Override
            public boolean update() {
                return false;
            }

            @Override
            public Line addLine() {
                return null;
            }

            @Override
            public Line addLine(String lineCaption) {
                return null;
            }

            @Override
            public boolean removeLine(Line line) {
                return false;
            }

            @Override
            public boolean isAutoClear() {
                return false;
            }

            @Override
            public void setAutoClear(boolean autoClear) {

            }

            @Override
            public int getMsTransmissionInterval() {
                return 0;
            }

            @Override
            public void setMsTransmissionInterval(int msTransmissionInterval) {

            }

            @Override
            public String getItemSeparator() {
                return "";
            }

            @Override
            public void setItemSeparator(String itemSeparator) {

            }

            @Override
            public String getCaptionValueSeparator() {
                return "";
            }

            @Override
            public void setCaptionValueSeparator(String captionValueSeparator) {

            }

            @Override
            public void setDisplayFormat(DisplayFormat displayFormat) {

            }

            @Override
            public Log log() {
                return null;
            }
        };

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setPIDF(kP, kI, kD, kF);

    }



    public void ShootBallLoop() {
        while (launcher.getVelocity()<velocityWanted || launcher.getVelocity()>velocityWanted) {

            double error = pidf.calculate(
                    launcher.getVelocity(), velocityWanted
            );

            launcher.setPower(error);
            telemetry.addData("Launcher Velocity", launcher.getVelocity());
            telemetry.update();
        }
    }
}
