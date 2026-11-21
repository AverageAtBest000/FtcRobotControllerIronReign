package org.firstinspires.ftc.teamcode.robots.bumblebee;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Flywheel;

@Config(value = "BumblebeeAuton")
@Autonomous(name = "BumbleBee", group = "BumbleBee")
public class BumblebeeAuton extends OpMode {

    public static double driveDelay_ms = 0;
    public static double drivePower = 0.4;
    public static double driveTime_ms = 1000;
    public static double shootTimeout_ms = 6000;

    private enum Stage { SHOOT, WAIT, DRIVE, DONE }

    private Robot robot;
    private Stage stage = Stage.SHOOT;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void start() {
        robot.flywheel.setBehavior(Flywheel.Behavior.LAUNCHING);
        stage = Stage.SHOOT;
        timer.reset();
    }

    @Override
    public void loop() {
        switch (stage) {
            case SHOOT:
                if (robot.flywheel.getBehavior() != Flywheel.Behavior.LAUNCHING
                        || timer.milliseconds() > shootTimeout_ms) {
                    robot.flywheel.setBehavior(Flywheel.Behavior.STOPPED);
                    stage = Stage.WAIT;
                    timer.reset();
                }
                break;

            case WAIT:
                if (timer.milliseconds() >= driveDelay_ms) {
                    stage = Stage.DRIVE;
                    timer.reset();
                }
                break;

            case DRIVE:
                robot.driveTrain.drive(drivePower, 0, 0);
                if (timer.milliseconds() >= driveTime_ms) {
                    robot.driveTrain.drive(0, 0, 0);
                    stage = Stage.DONE;
                }
                break;

            case DONE:
                robot.driveTrain.drive(0, 0, 0);
                break;
        }

        robot.readSensors();
        robot.calc(new Canvas());
        robot.act();

        telemetry.addData("Stage", stage);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
