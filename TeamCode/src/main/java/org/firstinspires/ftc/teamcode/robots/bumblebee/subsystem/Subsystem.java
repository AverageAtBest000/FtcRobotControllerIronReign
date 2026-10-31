package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

/**
 * Base interface for all robot subsystems.
 *
 * A subsystem is an independent unit that manages a specific part of the robot.
 * Each subsystem:
 * - Owns specific hardware (motors, servos, sensors)
 * - Has its own state machine (if needed)
 * - Provides telemetry for debugging
 * - Updates via the three-phase cycle
 *
 * The Robot class coordinates subsystems and handles cross-subsystem behaviors.
 *
 * THREE-PHASE UPDATE CYCLE:
 * Every loop, Robot calls these methods in order for ALL subsystems:
 *
 *   Phase 1: readSensors() - Read I2C sensors, cache values
 *   Phase 2: calc()        - State machines, PID, logic (use cached values)
 *   Phase 3: act()         - Flush lazy motor/servo commands to hardware
 *
 * This pattern ensures:
 * - All sensor reads happen before any calculations (no stale data)
 * - All calculations complete before any hardware writes
 * - I2C reads are consolidated for efficiency
 *
 * JUNIOR CODER RULES:
 * - Put all your code in calc()
 * - Use cachedSensor.getValue() not sensor.getValue()
 * - Use lazyMotor.setPower() not motor.setPower()
 * - Never call refresh() or flush() directly
 */
public interface Subsystem extends TelemetryProvider {

    /**
     * PHASE 1: Read all I2C sensors owned by this subsystem.
     *
     * Called once per loop BEFORE calc() for ALL subsystems.
     *
     * DO:
     * - Call refresh() on CachedDistanceSensor, LazyMotor (for current), etc.
     * - Update any cached sensor values
     *
     * DON'T:
     * - Run any logic or state machines
     * - Write to motors or servos
     * - Read from other subsystems
     */
    void readSensors();

    /**
     * PHASE 2: Run state machines and calculations.
     *
     * Called once per loop AFTER readSensors() for ALL subsystems.
     *
     * DO:
     * - Read cached sensor values
     * - Run state machine logic
     * - Calculate PID outputs
     * - Queue motor/servo commands via LazyMotor.setPower(), LazyServo.setPosition()
     *
     * DON'T:
     * - Read I2C sensors directly (use cached values)
     * - Write directly to motor.setPower() (use lazy wrappers)
     * - Call Thread.sleep() or blocking operations
     *
     * @param fieldOverlay Canvas for drawing on FTC Dashboard field view
     */
    void calc(Canvas fieldOverlay);

    /**
     * PHASE 3: Flush pending motor/servo commands to hardware.
     *
     * Called once per loop AFTER calc() for ALL subsystems.
     *
     * DO:
     * - Call flush() on LazyMotor, LazyServo
     *
     * DON'T:
     * - Run any logic
     * - Read any sensors
     */
    void act();

    /**
     * Called when the OpMode stops.
     *
     * Use this to:
     * - Set motors to zero power (directly, bypassing lazy)
     * - Move servos to safe positions
     * - Clean up any resources
     */
    void stop();

    /**
     * Reset all state machines to their initial states.
     *
     * Called when transitioning between autonomous and teleop,
     * or when the driver wants to cancel an in-progress action.
     */
    void resetStates();
}
