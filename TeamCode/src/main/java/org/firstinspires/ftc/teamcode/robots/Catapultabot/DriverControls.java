package org.firstinspires.ftc.teamcode.robots.Catapultabot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Catapult;
import org.firstinspires.ftc.teamcode.robots.Catapultabot.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class DriverControls {
    private final Robot catapultabot;
    private final Gamepad remote_control;
    private final StickyGamepad remote_toggle;

    public DriverControls(Robot robot, Gamepad gamepad) {
        catapultabot = robot;
        remote_control = gamepad;
        remote_toggle = new StickyGamepad(gamepad);
    }

    public void update(){
        activateDriveTrain();
        activateIntake();
        activateCatapult();
    }
    public void activateDriveTrain(){
        double forward = -remote_control.left_stick_y;
        double strafe = remote_control.left_stick_x;
        double turn = remote_control.right_stick_x;

        catapultabot.drive_train.setPower(forward, strafe, turn);
    }
    public void activateCatapult(){
        remote_toggle.update();
        if(remote_toggle.right_trigger){
            if(catapultabot.catapult.state == Catapult.catapult_state.ready)
                catapultabot.catapult.state = Catapult.catapult_state.readyer;
        }
    }
    public void activateIntake(){
        remote_toggle.update();

        if(remote_toggle.right_bumper){
            if(catapultabot.intake.getIntakeState() == Intake.Behavior.In)
                catapultabot.intake.setIntakeState(Intake.Behavior.Idle);
            else
                catapultabot.intake.setIntakeState(Intake.Behavior.In);
        }else if(remote_toggle.left_bumper) {
            if (catapultabot.intake.getIntakeState() == Intake.Behavior.Out)
                catapultabot.intake.setIntakeState(Intake.Behavior.Idle);
            else
                catapultabot.intake.setIntakeState(Intake.Behavior.Out);
        }
    }
}
