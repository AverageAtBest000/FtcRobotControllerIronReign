package org.firstinspires.ftc.teamcode.robots.Catapultabot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.drivetrain.DriveTrainBase;

import java.util.Collections;
import java.util.Map;

public class MeccanumDrive implements Subsystem
{
    //4 motors for 4 wheels
    private final DcMotorEx fleft, fright,  bleft, bright;
    private double forward, strafe, turn;
    private double flp, frp, blp, brp;
    public MeccanumDrive(HardwareMap controller)
    {
        //assign vars
        fleft = controller.get(DcMotorEx.class, "fleft");
        fright = controller.get(DcMotorEx.class, "fright");
        bleft = controller.get(DcMotorEx.class, "bleft");
        bright = controller.get(DcMotorEx.class, "bright");

        fleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);
    }
    //Sets motor power based on controll input+maths to drive
    public void drive(double forward, double strafe, double turn) {
            //math for moving
            double fl, fr, bl, br;
            fl = forward + strafe + turn;
            fr = forward - strafe - turn;
            bl = forward - strafe + turn;
            br = forward + strafe - turn;
            //math for mapping to -1 to 1 on a motor if they are greater than 1
            double max = Math.max(
                       1.0, Math.max(
                                     Math.max(fl, fr), Math.max(bl,br)
                                     )
                       );
            flp = fl/max;
            frp = fr/max;
            blp = bl/max;
            brp = br/max;
    }
    public void setPower(double f, double s, double t){
        forward = f;
        strafe = s;
        turn = t;
    }

    @Override
    public void readSensors() {
        return;
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        drive(forward, strafe, turn);
    }

    @Override
    public void act() {
        fleft.setPower(flp);
        fright.setPower(frp);
        bleft.setPower(blp);
        bright.setPower(brp);
    }

    @Override
    public void stop() {
        resetStates();
    }

    @Override
    public void resetStates() {
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "MeccanumDrive";
    }
}
