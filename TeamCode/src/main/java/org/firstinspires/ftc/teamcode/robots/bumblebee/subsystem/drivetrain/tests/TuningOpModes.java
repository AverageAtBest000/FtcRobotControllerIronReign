package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.tests;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain.PinpointLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final String GROUP = "bumblebee_rr";
    public static final String PREFIX = "6832 "; //XX indicates this is just an unintegrated template. change prefix to team number during integration
    public static final boolean DISABLED = false; // if needing to enable these, change this

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(PREFIX + cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    private static PinpointView makePinpointView(PinpointLocalizer pl) {
        return new PinpointView() {
            GoBildaPinpointDriver.EncoderDirection parDirection = pl.initialParDirection;
            GoBildaPinpointDriver.EncoderDirection perpDirection = pl.initialPerpDirection;

            @Override
            public void update() {
                pl.driver.update();
            }

            @Override
            public int getParEncoderPosition() {
                return pl.driver.getEncoderX();
            }

            @Override
            public int getPerpEncoderPosition() {
                return pl.driver.getEncoderY();
            }

            @Override
            public float getHeadingVelocity(UnnormalizedAngleUnit unit) {
                return (float) pl.driver.getHeadingVelocity(unit);
            }

            @Override
            public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                parDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            @Override
            public DcMotorSimple.Direction getParDirection() {
                return parDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }

            @Override
            public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                perpDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            @Override
            public DcMotorSimple.Direction getPerpDirection() {
                return perpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }
        };
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
  
            dvf = hardwareMap -> {
                MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                List<EncoderGroup> enccoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();

                PinpointView pinpointView = makePinpointView(md.localizer);
                enccoderGroups.add(new PinpointEncoderGroup(pinpointView));
                parEncs.add(new EncoderRef(0,0));
                perpEncs.add(new EncoderRef(0,1));

                return new DriveView(
                    DriveType.MECANUM,
                        MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.maxWheelVel,
                        MecanumDrive.PARAMS.minProfileAccel,
                        MecanumDrive.PARAMS.maxProfileAccel,
                        enccoderGroups,
                        Arrays.asList(
                                md.leftFront,
                                md.leftBack
                        ),
                        Arrays.asList(
                                md.rightFront,
                                md.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        new PinpointIMU(pinpointView),
                        md.voltageSensor,
                        () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                                MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),0
                );
            };

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
