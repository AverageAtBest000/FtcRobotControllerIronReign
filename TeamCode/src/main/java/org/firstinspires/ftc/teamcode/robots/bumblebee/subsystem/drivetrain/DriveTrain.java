package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.logging.Handler;

@Config(value = "BumbleBee_DriveTrain")
public class DriveTrain implements Subsystem {
    //  SET TO FALSE FOR ROBOT CENTRIC
    public boolean fieldCentric;  
    private MecanumDrive mecanumDrive;
    private PoseVelocity2d powers = new PoseVelocity2d( new Vector2d(0,0), 0);

    private double forward;
    private double strafe;
    private double turn;

    public DriveTrain(HardwareMap hardwareMap, Pose2d startPos, boolean fieldCentric){
        mecanumDrive = new MecanumDrive(hardwareMap, startPos);
        this.fieldCentric = fieldCentric; 
    }

    @Override
    public void readSensors() {
        mecanumDrive.localizer.refresh();
    }

    @Override
    public void calc(Canvas fieldOverlay) {  
        powers = new PoseVelocity2d(new Vector2d(forward, -strafe), -turn);        
    }

    @Override
    public void act() {
        mecanumDrive.setDrivePowers(powers);
    }

    @Override
    public void stop() {
        resetStates();
    }

    public Pose2d getPose(){
        return mecanumDrive.localizer.getPose();
    }

    @Override
    public void resetStates() {
    }

    public void drive(double forward, double strafe, double turn){
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Pose X: ", mecanumDrive.localizer.getPose().position.x);
        telemetry.put("Pose Y: ", mecanumDrive.localizer.getPose().position.y);
        
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "DriveTrain";
    }
}
