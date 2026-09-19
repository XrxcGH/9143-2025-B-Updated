package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.AlLowConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.AlLow;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.Vision;

/**
 * Binding-logic checks for the controller mapping (see the control map at
 * the top of RobotContainer). Nothing here depends on simulated motion: it
 * presses simulated buttons and looks at what the scheduler did.
 *
 * The camera-only driver bindings (align triggers, heading re-seed) are
 * checked in whichever state VisionConstants puts them: absent while no
 * Limelight is configured, live once one is.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class ControlsBindingTest {

    private RobotContainer container;
    private AlLow alLow;
    private KitBot kitBot;
    private XboxControllerSim driver;
    private XboxControllerSim operator;

    /** One container per JVM: the Spark MAX ids can only be claimed once. */
    @BeforeAll
    void setup() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        teleopEnabled();

        operator = new XboxControllerSim(ControllerConstants.OPERATOR_CONTROLLER_PORT);
        driver = new XboxControllerSim(ControllerConstants.DRIVER_CONTROLLER_PORT);
        neutral();
        container = new RobotContainer();
        alLow = (AlLow) field("alLow");
        kitBot = (KitBot) field("kitBot");
    }

    private void teleopEnabled() {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    /** Everything released. The simulated POV reads 0 (= up) until it is set, so center it. */
    private void neutral() {
        for (XboxControllerSim pad : new XboxControllerSim[] {driver, operator}) {
            for (int b = 1; b <= 10; b++) {
                pad.setRawButton(b, false);
            }
            for (int a = 0; a < 6; a++) {
                pad.setRawAxis(a, 0.0);
            }
            pad.setPOV(-1);
            pad.notifyNewData();
        }
    }

    @BeforeEach
    void reset() {
        teleopEnabled();
        neutral();
        run(2);
        CommandScheduler.getInstance().cancelAll();
        run(2);
    }

    @AfterAll
    void teardown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        HAL.shutdown();
    }

    private Object field(String name) throws Exception {
        Field f = RobotContainer.class.getDeclaredField(name);
        f.setAccessible(true);
        return f.get(container);
    }

    private void run(int loops) {
        for (int i = 0; i < loops; i++) {
            operator.notifyNewData();
            driver.notifyNewData();
            DriverStationSim.notifyNewData();
            CommandScheduler.getInstance().run();
        }
    }

    /** Runs scheduler loops for a stretch of wall-clock time (the trigger debouncer runs on the FPGA clock). */
    private void runFor(double seconds) throws InterruptedException {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            run(1);
            Thread.sleep(20);
        }
    }

    private Command driveCommand() {
        return container.swerve.getCurrentCommand();
    }

    // ------------------------------------------------------------------
    // Driver
    // ------------------------------------------------------------------

    @Test
    void sticksDriveByDefault() {
        run(2);
        assertEquals(container.swerve.getDefaultCommand(), driveCommand(), "the stick command owns the drivetrain at rest");
    }

    /** Hold-to-align: a trigger takes the drivetrain only while held - and not at all while no camera is configured. */
    @Test
    void alignTriggersAreHoldsAndExistOnlyWithACamera() {
        Command sticks = container.swerve.getDefaultCommand();
        for (boolean left : new boolean[] {true, false}) {
            if (left) {
                driver.setLeftTriggerAxis(1.0);
            } else {
                driver.setRightTriggerAxis(1.0);
            }
            run(3);
            if (Vision.hasCameras()) {
                assertNotEquals(sticks, driveCommand(), "a held align trigger runs the tracking command");
                assertTrue(container.swerve.isVisionTrackingEnabled());
            } else {
                assertEquals(sticks, driveCommand(), "no camera configured: the align triggers are not bound");
                assertFalse(container.swerve.isVisionTrackingEnabled());
            }
            neutral();
            run(3);
            assertEquals(sticks, driveCommand(), "release = sticks back at once");
            assertFalse(container.swerve.isVisionTrackingEnabled(), "never a toggle: nothing stays armed after release");
        }
    }

    @Test
    void driverYReseedsTheHeadingOnlyWithACamera() {
        assertFalse(container.swerve.getVision().isReseedingHeading(), "no re-seed window before the press");
        driver.setYButton(true);
        run(3);
        assertEquals(Vision.hasCameras(), container.swerve.getVision().isReseedingHeading(),
            "driver Y = fuse MegaTag1 for the re-seed window, once a camera exists");
    }

    @Test
    void pointWheelsAndSysIdExistOnlyInTestMode() {
        Command sticks = container.swerve.getDefaultCommand();
        driver.setBButton(true);
        run(3);
        assertEquals(sticks, driveCommand(), "B does nothing in teleop");
        neutral();
        driver.setRawButton(7, true); // Back
        driver.setYButton(true);
        run(3);
        assertEquals(sticks, driveCommand(), "Back + Y (SysId) does nothing in teleop");
        neutral();
        run(2);

        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        driver.setBButton(true);
        run(3);
        assertNotEquals(sticks, driveCommand(), "B points the wheels in Test mode");
        neutral();
        run(3);
        assertEquals(sticks, driveCommand());
    }

    @Test
    void dpadNudgesOnDiagonalsToo() {
        Command sticks = container.swerve.getDefaultCommand();
        driver.setPOV(45);
        run(3);
        assertNotEquals(sticks, driveCommand(), "a diagonal D-pad press still nudges");
        driver.setPOV(-1);
        run(3);
        assertEquals(sticks, driveCommand());
    }

    @Test
    void leftBumperZeroesOnlyTheDriversFrameWhileEnabled() {
        var before = container.swerve.getStateCopy().Pose.getRotation();
        driver.setLeftBumperButton(true);
        run(3);
        assertEquals(before.getDegrees(), container.swerve.getStateCopy().Pose.getRotation().getDegrees(), 1e-6,
            "the pose estimator's heading is not the driver's to move mid-match");
        assertEquals(container.swerve.getDefaultCommand(), driveCommand(), "and the sticks keep the drivetrain");
    }

    // ------------------------------------------------------------------
    // Operator
    // ------------------------------------------------------------------

    @Test
    void kitBotButtonsRunTheirCommands() {
        operator.setBButton(true);
        run(2);
        assertEquals("EjectFirstPiece", kitBot.getCurrentCommand().getName());
        neutral();
        run(2);
        assertEquals("EjectFirstPiece", kitBot.getCurrentCommand().getName(), "a timed eject outlives the press");

        operator.setYButton(true);
        run(2);
        assertEquals("EjectStackedPiece", kitBot.getCurrentCommand().getName());
        neutral();
        operator.setXButton(true);
        run(2);
        assertEquals("RealignPiece", kitBot.getCurrentCommand().getName());
        neutral();
        run(2);
        assertEquals(null, kitBot.getCurrentCommand(), "re-align is a hold");
        operator.setAButton(true);
        run(2);
        assertEquals("JogPiece", kitBot.getCurrentCommand().getName());
        neutral();
        run(2);
        assertEquals(null, kitBot.getCurrentCommand(), "jog is a hold");
    }

    @Test
    void alLowPresetsSetTheirAngles() {
        operator.setPOV(180);
        run(2);
        assertEquals(AlLowConstants.PivotPresetAngles.INTAKE.getAngle(), alLow.getTargetAngle(), 1e-9, "D-pad down = intake");
        operator.setPOV(90);
        run(2);
        assertEquals(AlLowConstants.PivotPresetAngles.HOLD.getAngle(), alLow.getTargetAngle(), 1e-9, "D-pad right = hold");
        operator.setPOV(0);
        run(2);
        assertEquals(AlLowConstants.PivotPresetAngles.BASE.getAngle(), alLow.getTargetAngle(), 1e-9, "D-pad up = stow");
        assertTrue(alLow.isHoldingPosition(), "a preset leaves the closed loop holding");
    }

    @Test
    void alLowRollerTriggersAreHolds() {
        Command manual = alLow.getDefaultCommand();
        operator.setLeftTriggerAxis(1.0);
        run(2);
        assertNotEquals(manual, alLow.getCurrentCommand(), "LT runs the intake rollers");
        neutral();
        run(2);
        assertEquals(manual, alLow.getCurrentCommand());
        operator.setRightTriggerAxis(1.0);
        run(2);
        assertNotEquals(manual, alLow.getCurrentCommand(), "RT runs the eject");
        neutral();
        run(2);
        assertEquals(manual, alLow.getCurrentCommand());
    }

    @Test
    void manualStickTakesOverAndReleaseHolds() {
        alLow.setPivotAngle(AlLowConstants.PivotPresetAngles.HOLD.getAngle());
        operator.setRightY(-1.0); // forward on the stick = deploy
        run(3);
        assertTrue(alLow.isInManualMode(), "stick outside the deadband = manual");
        assertFalse(alLow.isHoldingPosition());
        operator.setRightY(0.0);
        run(3);
        assertFalse(alLow.isInManualMode());
        assertTrue(alLow.isHoldingPosition(), "releasing the stick hands back to the closed-loop hold");
    }

    /** The encoder zero moves every preset and both soft limits: disabled only, and only after an ALLOW_ZERO_HOLD_SECONDS hold. */
    @Test
    void pivotZeroNeedsDisabledAndAFullHold() throws InterruptedException {
        double marker = AlLowConstants.PivotPresetAngles.HOLD.getAngle();

        alLow.setPivotAngle(marker);
        operator.setStartButton(true);
        runFor(AlLowConstants.ALLOW_ZERO_HOLD_SECONDS + 0.5);
        assertEquals(marker, alLow.getTargetAngle(), 1e-9, "Start does nothing while ENABLED, however long it is held");
        neutral();
        run(2);

        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        run(2);
        alLow.setPivotAngle(marker);
        operator.setStartButton(true);
        runFor(AlLowConstants.ALLOW_ZERO_HOLD_SECONDS * 0.3);
        assertEquals(marker, alLow.getTargetAngle(), 1e-9, "a brushed Start does nothing");
        runFor(AlLowConstants.ALLOW_ZERO_HOLD_SECONDS + 0.2);
        assertEquals(0.0, alLow.getTargetAngle(), 1e-9, "Start held for the full hold time while disabled zeroes the pivot");
        assertFalse(alLow.isHoldingPosition(), "zeroing drops the closed loop first");
    }

    /** Disabling drops the closed loop; enabling must pick the arm up again where it is. */
    @Test
    void enablingHoldsTheArmWhereItIs() {
        container.disabledInit();
        assertFalse(alLow.isHoldingPosition());
        container.enabledInit();
        assertTrue(alLow.isHoldingPosition(), "the arm is held on enable without the operator touching anything");
        assertEquals(alLow.getPivotAngle(), alLow.getTargetAngle(), 0.5, "at the angle it is resting at");
    }
}
