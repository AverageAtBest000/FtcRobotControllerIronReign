package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Config(value = "BumbleBee_DriveTrain")
public class DriveTrain implements Subsystem {
    //  SET TO FALSE FOR ROBOT CENTRIC
    public static boolean fieldCentric = true;

    // MOTOR AND IMU DECLARATION
    private final DcMotorEx frontRight, frontLeft, rearRight, rearLeft;
    private final IMU imu;

    private double headingDegrees, forward, strafe, turn;

    private double frontRightPower, frontLeftPower, rearLeftPower, rearRightPower;

    public DriveTrain(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);


        imu = fieldCentric ? hardwareMap.get(IMU.class, "imu") : null;

        if (imu != null) {
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new
                    RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }
    }

    public void drive(double f, double s, double t ){
        forward = f;
        strafe = s;
        turn = t;
    }

    public void driveNonFieldCentric(double forward, double strafe, double turn){
         frontLeftPower = forward + strafe + turn;
         frontRightPower = forward - strafe - turn;
         rearRightPower = forward + strafe - turn;
         rearLeftPower= forward - strafe + turn;

        double maxPower = Math.max( 1.0,
                Math.max( Math.max(Math.abs(frontLeftPower), Math.abs(rearLeftPower)),
                        Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        rearLeftPower /= maxPower;
        rearRightPower /= maxPower;

    }

    public void driveFieldCentric(double forward, double strafe, double turn){
        double theta = Math.toDegrees(Math.atan2(forward, strafe));
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeDegrees(theta - headingDegrees);

        double newForward = r * Math.sin(Math.toRadians(theta));
        double newRight = r * Math.cos(Math.toRadians(theta));

        driveNonFieldCentric(newForward, newRight, turn);
    }

    @Override
    public void readSensors() {
        if (imu != null) {
            headingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        if(fieldCentric) {
            driveFieldCentric(forward, strafe, turn);
        } else{
            driveNonFieldCentric(forward, strafe, turn);
        }
    }

    @Override
    public void act() {
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);
    }

    @Override
    public void stop() {
        resetStates();
    }

    @Override
    public void resetStates() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        if(fieldCentric) telemetry.put("Heading", headingDegrees);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "DriveTrain";
    }
}
