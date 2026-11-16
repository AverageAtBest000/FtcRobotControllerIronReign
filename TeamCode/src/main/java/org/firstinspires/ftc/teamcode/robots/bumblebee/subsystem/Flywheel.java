package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.Collections;
import java.util.Map;

public class Flywheel implements Subsystem{


    private final DcMotorEx flywheel;

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

    }

    @Override
    public void act() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void resetStates() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "";
    }
}
