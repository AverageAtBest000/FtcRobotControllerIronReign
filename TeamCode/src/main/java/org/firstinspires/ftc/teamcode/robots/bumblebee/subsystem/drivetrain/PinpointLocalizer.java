package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.rrQuickStart.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * Pinpoint localizer with three-phase update support.
 *
 * This localizer supports the three-phase update pattern:
 * - Phase 1: Call refresh() to perform the I2C bulk read
 * - Phase 2: Call update() - uses cached data if refresh() was called
 *
 * The Pinpoint performs a single 40-byte I2C bulk read to get all odometry data.
 * This is already efficient, but we cache it to avoid double reads if both
 * Robot.update() and RoadRunner call update() in the same cycle.
 */
@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 1407; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 3844; // x position of the perpendicular encoder (in tick units)
    }

    private final double xOffsetMm = 0.0;
    private final double yOffsetMm = 0.0; 

    // Encoder scales (ticks per millimetre) for reference.
    private static final float goBILDA_SWINGARM_POD = 13.26291192f;
    private static final float goBILDA_4_BAR_POD    = 19.89436789f;

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    // Three-phase support: track if refresh() was called this cycle
    private boolean refreshedThisCycle = false;
    private PoseVelocity2d cachedVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(goBILDA_4_BAR_POD, DistanceUnit.MM);
        //driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        // Set offsets AFTER resetPosAndIMU in case reset clears them
        driver.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        
        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        // Get fresh reading from Pinpoint before computing transform
        // This ensures the transform uses the current encoder values,
        // preventing drift if there's been any motion or noise since last refresh
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            );
        }
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    /**
     * PHASE 1: Refresh sensor data via I2C bulk read.
     * Call this once per cycle before any update() calls.
     *
     * This performs the actual I2C communication with the Pinpoint device.
     */
    public void refresh() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(
                    driver.getPosX(DistanceUnit.INCH),
                    driver.getPosY(DistanceUnit.INCH),
                    driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            );
            Vector2d worldVelocity = new Vector2d(
                    driver.getVelX(DistanceUnit.INCH),
                    driver.getVelY(DistanceUnit.INCH)
            );
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);
            cachedVelocity = new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        refreshedThisCycle = true;
    }
    public int getPar(){
        return driver.getEncoderX();
    }

    /**
     * Mark end of cycle. Call this at the end of Robot.update().
     * Resets the refresh flag so next cycle will do a fresh read.
     */
    public void markCycleComplete() {
        refreshedThisCycle = false;
    }

    @Override
    public PoseVelocity2d update() {
        // If refresh() wasn't called this cycle, do the I2C read now
        // This provides backwards compatibility with code that calls update() directly
        if (!refreshedThisCycle) {
            refresh();
        }
        return cachedVelocity;
    }
}