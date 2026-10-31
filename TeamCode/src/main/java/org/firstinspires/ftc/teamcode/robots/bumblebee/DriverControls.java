package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

public class DriverControls {
    private final Robot robot;
    private final Gamepad gamepad1;
    private final StickyGamepad stickyGamepad1;

    public DriverControls(Robot robot, Gamepad gamepad1){
        this.robot = robot;
        this.gamepad1 = gamepad1;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    public void update(){
        handleDrivetrain();
        handleIntake();
    }

    public void handleDrivetrain(){
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        robot.driveTrain.drive(forward, strafe, turn);
    }

    public void handleIntake(){
        stickyGamepad1.update();

        if(stickyGamepad1.right_bumper){
            if(robot.intake.getBehavior() == Intake.Behavior.INTAKING) robot.intake.setBehavior(Intake.Behavior.OFF);
            else robot.intake.setBehavior(Intake.Behavior.INTAKING);
        }else if(stickyGamepad1.left_bumper){
            if(robot.intake.getBehavior() == Intake.Behavior.EJECTING) robot.intake.setBehavior(Intake.Behavior.OFF);
            else robot.intake.setBehavior(Intake.Behavior.EJECTING);
        }
    }

}
