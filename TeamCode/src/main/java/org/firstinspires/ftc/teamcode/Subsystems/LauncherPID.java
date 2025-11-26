package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class LauncherPID  {

    public DcMotorEx launcher;

    public static double p = 1000, i = 0, d = 40, f = 45;

    public static int velocity;

    public int currentVelo;

    Telemetry telemetry;

    HardwareMap hardwareMap;




    public LauncherPID (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        velocity = -1565;


    }

    public void shooterLoop() {
        launcher.setVelocityPIDFCoefficients(p, i, d, f);
        launcher.setVelocity(velocity);

        currentVelo = (int)launcher.getVelocity();

        telemetry.addData("hello", currentVelo);
        telemetry.update();
    }

    public void StartShooter() {
        velocity = -1565;
    }

    public void StopShooter() {
        velocity = 40;
    }






}
