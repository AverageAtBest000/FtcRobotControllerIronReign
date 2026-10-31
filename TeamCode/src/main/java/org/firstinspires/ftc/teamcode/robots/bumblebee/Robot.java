package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class Robot implements Subsystem{

    public final DriveTrain driveTrain;
    public final Intake intake;

    private final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap hardwareMap){

        intake = new Intake(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);

        subsystems.add(driveTrain);
        subsystems.add(intake);

    }


    @Override
    public void readSensors() {
        // I2C SENSOR READ
        for(Subsystem subsystem : subsystems)
            subsystem.readSensors();


    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // RUN CALCULATIONS
        for (Subsystem subsystem : subsystems)
            subsystem.calc(fieldOverlay);
    }

    @Override
    public void act() {
        //FLUSH PENDING COMMANDS
        for (Subsystem subsystem : subsystems)
            subsystem.act();

    }

    @Override
    public void stop() {
        for(Subsystem subsystem : subsystems)
            subsystem.stop();
    }

    @Override
    public void resetStates() {
        for(Subsystem subsystem : subsystems)
            subsystem.resetStates();
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
