package org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.Map;

@Config(value = "catapultabot_intake")
public class Intake implements Subsystem{
    private final DcMotorEx spinner;
    private double IntakePower;
    public enum Behavior
    {
        Idle,
        Out,
        In,
    }
    public Behavior intakeState = Behavior.Idle;
    public static double IP = 0.425;
    //Need to change to velocity as that will change power on the fly, test later

    public Intake(HardwareMap controller)
    {
        spinner = controller.get(DcMotorEx.class, "intake");
    }

    public Behavior getIntakeState(){return intakeState;}
    public void setIntakeState(Behavior b){intakeState = b;}

    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        switch (intakeState){
            case In:
                IntakePower = IP;
                break;
            case Out:
                IntakePower = -IP;
                break;
            case Idle:
                IntakePower = 0;
                break;
        }
    }

    @Override
    public void act() {
        spinner.setPower(IntakePower);
    }

    @Override
    public void stop() {
        IntakePower = 0;
        spinner.setPower(IntakePower);
    }

    @Override
    public void resetStates() {
        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }
}
