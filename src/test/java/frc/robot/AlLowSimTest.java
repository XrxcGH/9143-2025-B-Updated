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
 * Exercises the AlLow pivot's desktop physics simulation: commands the
 * intake angle and checks that the simulated closed loop reaches and holds
 * it. The hold assertions are the regression net for the position-hold
 * design (reference latched on the Spark MAX; manual control only takes
 * over outside the stick deadband).
 */
class AlLowSimTest {

    private AlLow alLow;

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

        alLow = new AlLow();
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
        alLow.periodic();
        alLow.simulationPeriodic();
        SimHooks.stepTiming(0.02);
    }

    @Test
    void closedLoopReachesAndHoldsAngle() {
        assertEquals(0.0, alLow.getPivotAngle(), 1.0, "Arm must start stowed");

        double target = AlLowConstants.PivotPresetAngles.INTAKE.getAngle();
        alLow.setPivotAngle(target);

        // 10 simulated seconds for the P-only loop to settle
        for (int i = 0; i < 500; i++) {
            runLoop();
        }

        assertEquals(target, alLow.getPivotAngle(), AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR,
            "Simulated closed loop should reach the commanded angle");

        // A centered manual stick (the default command's steady state) must
        // not disturb the hold (the failure this test guards against).
        for (int i = 0; i < 250; i++) {
            alLow.manualPivotControl(0.0);
            runLoop();
        }

        assertEquals(target, alLow.getPivotAngle(), AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR,
            "Arm should keep holding the angle while the manual stick is centered");
    }
}
