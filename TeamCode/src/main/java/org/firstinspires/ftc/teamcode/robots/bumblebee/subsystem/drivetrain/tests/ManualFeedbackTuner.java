package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.PinpointLocalizer;

@Config("Bumblebee_ManualFeedbackTuner")   
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 48;    

    @Override
    public void runOpMode() throws InterruptedException {
        if (PinpointLocalizer.PARAMS.parYTicks == 0 && PinpointLocalizer.PARAMS.perpXTicks == 0) {
            throw new RuntimeException("Pinpoint pod offsets not set! Run AngularRampLogger first.");
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
        }
    }
}