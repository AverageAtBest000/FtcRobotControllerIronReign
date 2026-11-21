package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "BumbleBee_Ramp")
public class Ramp implements Subsystem{

    // SERVO DECLARATION
    private final Servo ramp;

    // SERVO POSITIONS (TUNE IN DASHBOARD)
    public static double loweredPosition = 0.0;
    public static double raisedPosition = 1.0;

    // RAMP STATES
    public static enum Behavior {
        LOWERED,
        RAISED
    }

    private Behavior behavior = Behavior.LOWERED;

    // TO FLUSH
    private double rampPosition = loweredPosition;

    public Ramp(HardwareMap hardwareMap){
        ramp = hardwareMap.get(Servo.class, "ramp");
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay){
        switch (behavior){

            case LOWERED:
                rampPosition = loweredPosition;
                break;

            case RAISED:
                rampPosition = raisedPosition;
                break;
        }
    }

    @Override
    public void act(){
        ramp.setPosition(rampPosition);
    }

    @Override
    public void stop(){
        behavior = Behavior.LOWERED;
    }

    public void setBehavior(Behavior behavior){
        this.behavior = behavior;
    }

    public Behavior getBehavior(){
        return behavior;
    }

    @Override
    public void resetStates(){
        behavior = Behavior.LOWERED;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Ramp Position", rampPosition);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Ramp";
    }

}
