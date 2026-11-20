package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

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
    public static boolean fieldCentric = true;  
    private MecanumDrive mecanumDrive;
    private PoseVelocity2d powers = new PoseVelocity2d( new Vector2d(0,0,0));

    double forward;
    double strafe;
    double turn;

    public DriveTrain(HardwareMap hardwareMap, Pose2d startPos, boolean isFieldCentric){
        mecanumDrive = new MecanumDrive(hardwareMap, startPos);
        this.isFieldCentric = isFieldCentric; 
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

        telemetry.put("Pose X: ", mecanumDrive.localizer.getPose().getPosX);
        telemetry.put("Pose Y: ", mecanumDrive.localizer.getPose().getPosY);
        
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "DriveTrain";
    }
}
