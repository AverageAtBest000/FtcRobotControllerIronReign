package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "BumbleBee_Flywheel")
public class Flywheel implements Subsystem{


    private final DcMotorEx flywheel;

    public static double goalRPM = 700;

    private double bareTicksPerRev  = 28.0;
    private double gearRatio = 48.0/30.0;
    private double ticksPerRev = bareTicksPerRev * gearRatio;
    private double ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;


    public static enum Behavior{
        STOPPED,
        RUNNING
    }

    private  Behavior behavior = Behavior.STOPPED;

    public Flywheel(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {

        switch(behavior){
            case STOPPED:
                ticksPerSecond = 0;
            break;

            case RUNNING:
                ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;
            break;
        }

    }

    @Override
    public void act() {
        flywheel.setVelocity(ticksPerSecond);
    }

    @Override
    public void stop() {
        flywheel.setPower(0);
    }

    public Behavior getBehavior(){
        return behavior; 
    }

    public void setBehavior(Behavior behavior){
        this.behavior = behavior;
    }

    @Override
    public void resetStates() {
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetry = new LinkedHashMap<>();
        
        telemetry.put("Flywheel RPM: ", (flywheel.getVelocity() * 60.0) / ticksPerRev );
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Flywheel";
    }
}
