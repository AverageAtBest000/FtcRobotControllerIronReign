package org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.Map;

@Config(value = "Catapultabot_Flywheel")
public class Flywheel implements Subsystem{
    public static enum FlyBehavior{
        STOPPED,
        IDLE,
        LAUNCHING
    }
    public FlyBehavior behavior = FlyBehavior.STOPPED;

    private final DcMotorEx motor;
    private final DcMotorEx feeder;

    //flywheel math & vars

    //bareTicksPerRev * gearRatio of motor = ticks per rev
    //ticksPerSercond = (ticksPerRev * goalRpm) / 60
    public static double goalRPM = 3700;
    public static double currentRPM = 0;
    private double ticksPerRev = 28.0 * ( 48.0/30.0 );
    public static double idlePercent = 0.2;
    private double ticksPerSecond = ( ticksPerRev * goalRPM ) / 60.0;
    private static long spinUpTimeout_ms = 2000L;
    //note: to save battery, need to make it so it revs up when it's fed, and is "powered down" otherwise

    //feeder vars
    public static double feedPower = 1.0;
    private double currentFeedPower = 0.0;
    private boolean feeding = false;
    public static long feedTime_ms = 4000L;

    //other vars
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private double lastSpinUpTime_ms = 0.0;
    public static double rpmTolerence = 25;

    //pulse to launch one at a time
    public static boolean pulse = false;
    public static long pulseTime_ms = 100L;
    private long lastPulseStart_ms = 0L;
    private boolean pulseOn = true;

    public Flywheel(HardwareMap hubController){
        motor = hubController.get(DcMotorEx.class, "flywheel");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        feeder = hubController.get(DcMotorEx.class, "flywheel");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        currentRPM = ( motor.getVelocity() * 60 ) / ticksPerRev;
        switch(behavior){
            case STOPPED:
                ticksPerSecond = 0;
                break;
            case IDLE:
                ticksPerSecond = ( ticksPerRev * goalRPM * idlePercent) / 60.0;
                break;
            case LAUNCHING:
                ticksPerSecond = ( ticksPerRev * goalRPM ) / 60.0;

                boolean atSpeed = Math.abs(currentRPM - goalRPM) <= rpmTolerence;
                boolean spinUpAndTimeOut = spinUpTimer.milliseconds() >= spinUpTimeout_ms;

                if(!feeding && ( atSpeed || spinUpAndTimeOut ) ){
                    feeding = true;
                    feedTimer.reset();
                    lastSpinUpTime_ms = spinUpTimer.milliseconds();
                    pulseOn = true;
                    lastPulseStart_ms = System.nanoTime() / 1_000_000;
               }

                if(feeding){
                    if(!pulse){
                        currentFeedPower = feedPower;
                    }else{
                        long timeNow_ms = System.nanoTime() / 1_000_000;

                        if(pulseOn){
                            currentFeedPower = feedPower;

                            if(timeNow_ms - lastPulseStart_ms > pulseTime_ms){
                                pulseOn = false;
                            }

                        }else{
                            currentFeedPower = 0;
                            if(Math.abs(currentRPM - goalRPM) >= rpmTolerence){
                                pulseOn = true;
                                lastPulseStart_ms = timeNow_ms;
                            }
                        }
                    }

                    if(feedTimer.milliseconds() >= feedTime_ms){
                        behavior = FlyBehavior.IDLE;
                        ticksPerSecond = (ticksPerRev * goalRPM * idlePercent) / 60.0;
                        currentFeedPower = 0;
                        feeding = false;
                    }
                }else{
                    currentFeedPower = 0;
                }

                break;
        }
    }

    @Override
    public void act() {
        motor.setVelocity(ticksPerSecond);
        feeder.setPower(currentFeedPower);
    }

    @Override
    public void stop() {
        motor.setPower(0);
        feeder.setPower(0);
        feeding = false;
        behavior = FlyBehavior.STOPPED;
    }
    public FlyBehavior getBehavior(){
        return behavior;
    }
    public void setBehavior(FlyBehavior behavior){
        if(behavior == FlyBehavior.LAUNCHING && this.behavior != FlyBehavior.LAUNCHING){
            feeding = false;
            spinUpTimer.reset();
        }
        this.behavior = behavior;
    }


    @Override
    public void resetStates() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "Flywheel";
    }
}
