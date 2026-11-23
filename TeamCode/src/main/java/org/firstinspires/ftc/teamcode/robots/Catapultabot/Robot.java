package org.firstinspires.ftc.teamcode.robots.Catapultabot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Catapult;
import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Subsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class Robot implements Subsystem
{
    public final MeccanumDrive drive_train;
    public final Intake intake;
    //public final Catapult catapult;

    public final List<Subsystem> subsystems = new ArrayList<>();

    public Robot(HardwareMap controller)
    {
        drive_train = new MeccanumDrive(controller);
        intake = new Intake(controller);
        //catapult = new Catapult(controller);

        subsystems.add(drive_train);
        subsystems.add(intake);
        //subsystems.add(catapult);
    }

    @Override
    public void readSensors() {
        for(Subsystem subsystem : subsystems)
            subsystem.readSensors();
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        for(Subsystem subsystem : subsystems)
            subsystem.calc(fieldOverlay);
    }

    @Override
    public void act() {
        for(Subsystem subsystem : subsystems)
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
        return "Catapultabot";
    }
}
