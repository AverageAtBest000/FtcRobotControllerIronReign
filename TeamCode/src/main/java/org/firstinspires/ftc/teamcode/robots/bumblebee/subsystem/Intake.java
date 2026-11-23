package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "BumbleBee_Intake")
public class Intake implements Subsystem{


    // MOTOR DECLARATION
    private final DcMotorEx intake;
    private DcMotorEx intakeSlave;
    private boolean usingSlave;

    // ENCODER CALCULATION VARIABLES
    public static double goalRPM = 1400;

    private double bareTicksPerRev  = 28.0;
    private double gearRatio = 3.0/1.0;
    private double ticksPerRev = bareTicksPerRev * gearRatio;
    private double ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;

    //INTAKING STATES
    public static enum Behavior {
        OFF,
        INTAKING,
        EJECTING
    }
    
    private Behavior behavior = Behavior.OFF;

    // TO FLUSH
    private double intakeVelocity = 0.0;
    
    public Intake(HardwareMap hardwareMap, boolean usingSlave){

        this.usingSlave = usingSlave;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        if(usingSlave){
            intakeSlave = hardwareMap.get(DcMotorEx.class, "intakeSlave");
            intakeSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay){

        ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;
        switch (behavior){

            case OFF:
                intakeVelocity = 0.0;
                break;

            case INTAKING:
                intakeVelocity = ticksPerSecond;
                break;

            case EJECTING:
                intakeVelocity = -ticksPerSecond;
                break;
        }

    }

    @Override
    public void act(){
        intake.setVelocity(intakeVelocity);
        if(usingSlave) intakeSlave.setVelocity(intakeVelocity);
    }

    @Override
    public void stop(){
        behavior = Behavior.OFF;
        intake.setPower(0.0);
        if(usingSlave) intakeSlave.setPower(0.0);
    }

    public void setBehavior(Behavior behavior){
        this.behavior = behavior;
    }

    public Behavior getBehavior(){
        return behavior;
    }

    @Override
    public void resetStates(){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        if(usingSlave){
          intakeSlave.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          intakeSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Goal RPM", goalRPM);
        telemetry.put("Intake RPM", intake.getVelocity() * 60.0 / ticksPerRev);
        telemetry.put("Intake Slave RPM", (usingSlave)? intakeSlave.getVelocity() * 60.0 / ticksPerRev : null);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }

}

