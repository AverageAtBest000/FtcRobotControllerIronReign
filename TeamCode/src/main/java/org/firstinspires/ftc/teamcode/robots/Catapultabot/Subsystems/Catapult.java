package org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Catapultabot.DriverControls;

import java.util.Collections;
import java.util.Map;

public class Catapult implements Subsystem{
    private final DcMotorEx setter;
    private final double ticks_per_rotation;
    private int total_ticks;
    private final double rotations;
    private final Servo releaser;
    private final double servo_open;
    private final double servo_closed;
    private enum catapult_state{
        ready,
        launch,
        launched,
        resetting,
        reset,
    }
    private catapult_state state = catapult_state.ready;
    public Catapult(HardwareMap controller)
    {
        //NA for now, no motors to reset catapult
        setter = controller.get(DcMotorEx.class, "catapult_reset");
        releaser = controller.get(Servo.class, "catapult_release");
        //a dc motor with a encoder for positions to turn to
        setter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //get ticks per rotation, tell motor num of rotations per num of ticks
        ticks_per_rotation = 60.00;
        rotations = 1.5;
        total_ticks = (int)(-ticks_per_rotation*rotations);

        //servo goes from 0 to 1; 0 to 180 degrees
        servo_open = 0.0;
        servo_closed = 0.5;
        setter.setPower(0.5);
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        switch(state) {
            case ready:
                setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                total_ticks=0;
                break;
            case launched:
                setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                total_ticks=(int)(-ticks_per_rotation*rotations);
                break;
            case reset:
                setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                total_ticks= (int)(ticks_per_rotation*rotations);
                break;
            default:
                total_ticks=0;
                break;
        }
    }

    @Override
    public void act() {
        //do work
        switch(state) {
            case ready:
                state=catapult_state.launch;
                break;
            case launch:
                releaser.setPosition(servo_open);
                if(releaser.getPosition()==servo_open)
                    state=catapult_state.launched;
                break;
            case launched:
                setter.setTargetPosition(total_ticks);
                setter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(!setter.isBusy())
                    state=catapult_state.resetting;
                break;
            case resetting:
                setter.setTargetPosition(total_ticks);
                setter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(!setter.isBusy())
                    state=catapult_state.reset;
                break;
            case reset:
                releaser.setPosition(servo_closed);
                setter.setTargetPosition(total_ticks);
                setter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(!setter.isBusy() && releaser.getPosition()==servo_closed)
                    state=catapult_state.ready;
                break;
            default:
                break;
        }
    }

    @Override
    public void stop() {
        setter.setPower(0);
    }

    @Override
    public void resetStates() {
        setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        releaser.setPosition(servo_closed);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "Catapult";
    }
}
