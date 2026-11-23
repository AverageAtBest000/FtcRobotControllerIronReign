package org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.Catapultabot.DriverControls;

import java.util.Collections;
import java.util.Map;

@Config(value = "catapultabot_catapult")
public class Catapult implements Subsystem{
    private final DcMotorEx setter;
    private final double ticks_per_rotation;
    private int total_ticks;
    private final double rotations;
    private final Servo releaser;
    private final double servo_open;
    private final double servo_closed;
    public enum catapult_state{
        ready,
        readyer,
        launch,
        launched,
        resetting,
        reset,
    }
    public catapult_state state = catapult_state.ready;
    public Catapult(HardwareMap controller)
    {
        //NA for now, no motors to reset catapult
        setter = controller.get(DcMotorEx.class, "catapult_reset");
        releaser = controller.get(Servo.class, "catapult_release");
        //a dc motor with a encoder for positions to turn to
        setter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //get ticks per rotation, tell motor num of rotations per num of ticks
        //base encoder ticks times total gear ratio (2 X 5.83:1) gears
        ticks_per_rotation = ( 28 * 2 * 5.23 );
        //ticks_per_rotation = ( 28);
        rotations = 1;
        total_ticks = (int)(-ticks_per_rotation*rotations);

        //servo goes from 0 to 1; 0 to 180 degrees., but represented as a range from ~900 ticks to ~2048 ticks
        servo_closed = servoNormalize(899);
        servo_open = servoNormalize(1845);
        releaser.setPosition(servo_open);
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0;
    }

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        switch(state) {
            case ready:
                break;
            case launched:
                total_ticks=(int)(-ticks_per_rotation*rotations);
                break;
            case reset:
                total_ticks=0;
                break;
            default:
                break;
        }
    }

    @Override
    public void act() {
        //do work
        switch(state) {
            case ready:
                break;
            case readyer:
                state=catapult_state.launch;
                break;
            case launch:
                releaser.setPosition(servo_open);
                state = catapult_state.launched;
                break;
            case launched:
                setter.setTargetPosition(total_ticks);
                setter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(!setter.isBusy())
                    state=catapult_state.resetting;
                break;
            case resetting:
                releaser.setPosition(servo_closed);
                state = catapult_state.launched;
                break;
            case reset:
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
        //setter.setPower(0);
    }

    @Override
    public void resetStates() {
        setter.setTargetPosition(0);
        setter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        releaser.setPosition(servo_open);
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
