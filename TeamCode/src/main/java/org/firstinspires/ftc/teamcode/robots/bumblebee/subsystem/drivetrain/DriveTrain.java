package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
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
    //  SET TO FALSE FOR FIELD CENTRIC
    public boolean isRobotCentric;

    // ======================= DRIVE DECLARATIONS =====================================
    private final MecanumDrive mecanumDrive;
    private PoseVelocity2d powers = new PoseVelocity2d( new Vector2d(0,0), 0);

    private double forward;
    private double strafe;
    private double turn;

    private Action action = null; 

    public static enum Behavior{
        MANUAL,
        TRAJECTORY
    }

    Behavior behavior = Behavior.MANUAL;


    public DriveTrain(HardwareMap hardwareMap, Pose2d startPos, boolean isRobotCentric){
        mecanumDrive = new MecanumDrive(hardwareMap, startPos);
        this.isRobotCentric = isRobotCentric;
    }

    @Override
    public void readSensors() {
        mecanumDrive.localizer.refresh();
    }

    @Override
    public void calc(Canvas fieldOverlay) {  
        
        switch (behavior) {
            case MANUAL:
                powers = new PoseVelocity2d(new Vector2d(forward, -strafe), -turn);                        
                break;
        
            case TRAJECTORY:

                if(!action.run(new TelemetryPacket())){
                    action = null;
                    behavior = Behavior.MANUAL;
                }

                break;
        }
    
    }

    @Override
    public void act() {
        if(behavior != Behavior.TRAJECTORY)
            mecanumDrive.setDrivePowers(powers);
    }


    @Override
    public void stop() {
        action = null;
        behavior = Behavior.MANUAL; 
        powers = new PoseVelocity2d(new Vector2d(0.0,0.0), 0.0);
        mecanumDrive.setDrivePowers(powers);
    }

    public Pose2d getPose(){
        return mecanumDrive.localizer.getPose();
    }

    @Override
    public void resetStates() {
    }

    //============================ TELE-OP =============================
    public void drive(double forward, double strafe, double turn){
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {

        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                getPose().heading.toDouble());

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.forward = newForward;
        this.strafe = newStrafe;
        this.turn = turn;
    }

    //============================ TELEMETRY ===========================
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Pose X: ", getPose().position.x);
        telemetry.put("Pose Y: ", getPose().position.y);
        telemetry.put("Heading (deg): ", Math.toDegrees(getPose().heading.toDouble()));
        
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "DriveTrain";
    }
}
