package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

import frc.robot.Constants.AlLowConstants;
import frc.robot.subsystems.AlLow;

/**
 * Exercises the AlLow pivot's manual control against its desktop physics
 * simulation with a stepped clock: drives the arm from the stick, releases
 * it mid-travel and checks where the hold lands. (A class of its own: each
 * test class runs in its own JVM, and a Spark MAX id can only be claimed
 * once per process.)
 */
class AlLowManualReleaseSimTest {

    private AlLow allow;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        // Freeze the simulated FPGA clock and step it manually below, so the
        // on-controller closed loop (which runs on HAL time) advances in
        // lockstep with the physics updates - otherwise results depend on
        // how fast the test host executes the loop.
        SimHooks.pauseTiming();
        // The Spark MAX sim only drives outputs while the robot is enabled
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        allow = new AlLow();
    }

    @AfterEach
    void teardown() {
        SimHooks.resumeTiming();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        HAL.shutdown();
    }

    /** Runs one 20 ms robot loop's worth of subsystem + physics + clock updates. */
    private void runLoop() {
        DriverStationSim.notifyNewData();
        allow.periodic();
        allow.simulationPeriodic();
        SimHooks.stepTiming(0.02);
    }

    /**
     * Releasing the manual stick mid-travel must hold where the moving arm
     * can STOP - ahead of the angle it was passing through - and the target
     * is set once, not chased: the arm settles on it without being dragged
     * back.
     */
    @Test
    void manualReleaseHoldsWhereTheArmStops() {
        // Deploy under manual control for 0.2 simulated seconds: long enough to
        // be moving, short enough to be nowhere near the forward soft limit
        for (int i = 0; i < 10; i++) {
            allow.manualPivotControl(1.0);
            runLoop();
        }
        assertTrue(allow.isInManualMode());
        double angleAtRelease = allow.getPivotAngle();
        double velocityAtRelease = allow.getPivotVelocity();
        assertTrue(velocityAtRelease > 0.0, "the arm is still deploying when the stick is released");

        allow.manualPivotControl(0.0);
        double target = allow.getTargetAngle();
        assertTrue(allow.isHoldingPosition(), "release hands over to the closed loop at once");
        assertTrue(target >= angleAtRelease, "the hold target is ahead of a deploying arm, never behind it");
        assertEquals(angleAtRelease
                + velocityAtRelease * AlLowConstants.ALLOW_MANUAL_RELEASE_STOP_SECONDS / 2.0,
            target, 1.0, "measured angle plus the stopping distance (clamped to the travel limits)");

        for (int i = 0; i < 250; i++) {
            allow.manualPivotControl(0.0);
            runLoop();
        }
        assertEquals(target, allow.getTargetAngle(), 1e-9, "the target is set once on release");
        assertEquals(target, allow.getPivotAngle(), AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR,
            "and the arm settles on it");
    }
}
