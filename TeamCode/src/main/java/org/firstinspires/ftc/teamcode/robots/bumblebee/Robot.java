package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.roadrunner.Pose2d;


import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Ramp;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class Robot implements Subsystem{

    public final DriveTrain driveTrain;
    private boolean driveFieldCentric = false; 

    public final Intake intake;
    private boolean hasSlave = false;

    public final Flywheel flywheel;

    public final Ramp ramp;

    private final List<Subsystem> subsystems = new ArrayList<>();

    public static double startPosX = 0.0;
    public static double startPozY = 0.0;
    public static double startHeading = 0.0; 

    public Pose2d Pose = new Pose2d(startPosX, startPozY, startHeading);

    public Robot(HardwareMap hardwareMap){

        intake = new Intake(hardwareMap, hasSlave);
        driveTrain = new DriveTrain(hardwareMap, Pose, driveFieldCentric);
        flywheel = new Flywheel(hardwareMap);
        ramp = new Ramp(hardwareMap);

        subsystems.add(driveTrain);
        subsystems.add(intake);
        subsystems.add(flywheel);
        subsystems.add(ramp);
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

    public void updateTelemetry(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems)
            for (Map.Entry<String, Object> entry : subsystem.getTelemetry(false).entrySet())
                telemetry.addData(subsystem.getTelemetryName() + " " + entry.getKey(), entry.getValue());
        telemetry.update();
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
