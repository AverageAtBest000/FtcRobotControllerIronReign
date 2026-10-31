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
    private final DcMotorEx intakeSlave;


    // ENCODER CALCULATION VARIABLES
    public static double goalRPM = 700;
    public static double ticksPerSecond;


    //INTAKING STATES
    public static enum Behavior {
        OFF,
        INTAKING,

        EJECTING
    }
    private Behavior behavior = Behavior.OFF;

    // TO FLUSH
    private double intakeVelocity = 0.0;
    double ticksPerRev;

    public Intake(HardwareMap hardwareMap){

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intakeSlave= hardwareMap.get(DcMotorEx.class, "intakeSlave");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ticksPerRev = intake.getMotorType().getTicksPerRev();
        ticksPerSecond = (ticksPerRev * goalRPM) / 60.0;
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay){
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
        intakeSlave.setVelocity(intakeVelocity);
    }

    @Override
    public void stop(){
        behavior = Behavior.OFF;
        intake.setPower(0.0);
        intakeSlave.setPower(0.0);
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
        intakeSlave.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlave.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Goal RPM", goalRPM);
        telemetry.put("Intake RPM", intake.getVelocity() * 60.0 / ticksPerRev);
        telemetry.put("Intake Slave RPM", intakeSlave.getVelocity() * 60.0 / ticksPerRev);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }

}

