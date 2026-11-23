package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "BumbleBee_Flywheel")
public class Flywheel implements Subsystem{


    public static boolean pulse = false;
    public static long pulseDuration_ms = 100L;
    private long lastPulseStart_ms = 0L;
    private boolean pulseOn = true;

    private final DcMotorEx flywheel;
    private final DcMotorEx feeder;

    public static double goalRPM = 3700;
    public static double rpmTolerance = 25;
    public static long feedTime_ms = 4000L;
    public static long spinUpTimeout_ms = 2000L;
    public static double idlePercent = 0.5;

    private double bareTicksPerRev  = 28.0;
    private double gearRatio = 48.0/30.0;
    private double ticksPerRev = bareTicksPerRev * gearRatio;
    private double ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;

    public static double feedPower = 1.0;
    private double currentFeedPower = 0;

    private double currentRPM = 0;
    private boolean feeding = false;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double lastSpinUpTime_ms = 0;

    public static enum Behavior{
        STOPPED,
        IDLE,
        LAUNCHING
    }

    private  Behavior behavior = Behavior.STOPPED;

    public Flywheel(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        currentRPM = (flywheel.getVelocity() * 60.0) / ticksPerRev;

        switch(behavior){
            case STOPPED:
                ticksPerSecond = 0;
                currentFeedPower = 0;
                feeding = false;
            break;

            case IDLE:
                ticksPerSecond = (ticksPerRev * goalRPM * idlePercent) / 60.0;
                currentFeedPower = 0;
                feeding = false;
            break;

            case LAUNCHING:
                ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;

                boolean atSpeed = Math.abs(currentRPM - goalRPM) <= rpmTolerance;
                boolean spinUpTimedOut = spinUpTimer.milliseconds() >= spinUpTimeout_ms;

                if (!feeding && (atSpeed || spinUpTimedOut)) {
                    feeding = true;
                    feedTimer.reset();
                    lastSpinUpTime_ms = spinUpTimer.milliseconds();
                    pulseOn = true;
                    lastPulseStart_ms = System.nanoTime() / 1_000_000;
                }

                if (feeding) {
                    if(!pulse){
                        currentFeedPower = feedPower;
                    }else{
                        long timeNow_ms = System.nanoTime() / 1_000_000;

                        if(pulseOn){
                            currentFeedPower = feedPower;
                            if(timeNow_ms - lastPulseStart_ms > pulseDuration_ms){
                                pulseOn = false;
                            }
                        } else {
                            currentFeedPower = 0;
                            if(Math.abs(currentRPM - goalRPM) <= rpmTolerance) {
                                pulseOn = true;
                                lastPulseStart_ms = timeNow_ms;
                            }
                        }
                    }

                    if (feedTimer.milliseconds() >= feedTime_ms) {
                        behavior = Behavior.IDLE;
                        ticksPerSecond = (ticksPerRev * goalRPM * idlePercent) / 60.0;
                        currentFeedPower = 0;
                        feeding = false;
                    }
                } else {
                    currentFeedPower = 0;
                }
            break;
        }

    }

    @Override
    public void act() {
        flywheel.setVelocity(ticksPerSecond);
        feeder.setPower(currentFeedPower);
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
        feeder.setPower(0);
        feeding = false;
        behavior = Behavior.STOPPED;
    }

    public Behavior getBehavior(){
        return behavior; 
    }

    public void setBehavior(Behavior behavior){
        if (behavior == Behavior.LAUNCHING && this.behavior != Behavior.LAUNCHING) {
            feeding = false;
            spinUpTimer.reset();
        }
        this.behavior = behavior;
    }

    @Override
    public void resetStates() {
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetry = new LinkedHashMap<>();
        
        telemetry.put("Behavior: ", behavior);
        telemetry.put("Flywheel RPM: ", currentRPM);
        telemetry.put("Goal RPM: ", goalRPM);
        telemetry.put("Feeding: ", feeding);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Flywheel";
    }
}
