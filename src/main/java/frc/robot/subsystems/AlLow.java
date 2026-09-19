package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AlLowConstants;

/**
 * AlLow (Algae Low) ground intake: a pivoting arm with intake rollers, both
 * NEOs on Spark MAX controllers.
 *
 * Control architecture:
 *  - The encoder conversion factors scale the NEO's integrated encoder so
 *    every pivot position is in degrees and every velocity in degrees per
 *    second (0 degrees = stowed; the encoder is zeroed there on init).
 *  - Angle moves use closed-loop position control on the Spark MAX with a
 *    sin(angle) gravity feedforward passed as arbitrary feedforward voltage.
 *    The controller latches the reference, so the arm keeps holding its
 *    angle no matter which command is currently scheduled - roller-only
 *    commands and the manual default command never disturb the hold.
 *  - periodic() re-sends the reference while position control is active so
 *    the gravity term tracks the measured angle through the whole travel,
 *    not the angle the arm had when the target was set. That is safe
 *    because this is plain position control: the reference is a number the
 *    PID compares against, and sending the same number again changes
 *    nothing. It would not be safe under MAXMotion, which restarts its
 *    motion profile from the measured state on every new setpoint - there a
 *    setpoint must be sent once per move.
 *  - Manual stick control drives the motor open-loop; the moment the stick
 *    returns to the deadband the arm is held under closed loop, so it never
 *    goes limp mid-air. The hold target is where the moving arm can stop
 *    (see holdWhereTheArmStops), not the angle it is passing through.
 *  - The arm is also held where it is every time the robot enables
 *    (RobotContainer.enabledInit): disabling drops the closed loop, and
 *    nothing else would pick a deployed arm up again.
 *  - Soft limits on the controller bound travel in every control mode, and
 *    voltage compensation keeps response consistent as the battery sags.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class AlLow extends SubsystemBase {

    private final SparkMax pivotMotor;
    private final SparkMax rollerMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    // Track target angle internally (degrees)
    private double targetAngle = 0.0;
    // Track manual mode state internally
    private boolean manualModeEnabled = false;
    // Track position control mode internally
    private boolean positionControlEnabled = false;

    // ------------------------------------------------------------------
    // Desktop simulation (only constructed when running off-robot). The
    // physics model exists purely so the mechanism moves in the sim GUI /
    // AdvantageScope; its model values are ALLOW_ARM_LENGTH_METERS and the
    // ALLOW_SIM_ constants.
    // ------------------------------------------------------------------
    private SparkMaxSim pivotMotorSim;
    private SingleJointedArmSim armSim;

    public AlLow() {
        pivotMotor = new SparkMax(AlLowConstants.ALLOW_PIVOT_MOTOR_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(AlLowConstants.ALLOW_ROLLER_MOTOR_ID, MotorType.kBrushless);

        configureMotors();

        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getClosedLoopController();

        // Zero the pivot on initialization (arm must start at its stowed position)
        resetPivotEncoder();

        if (RobotBase.isSimulation()) {
            // getNEO(1): the one NEO a Spark MAX drives
            pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
            armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                AlLowConstants.ALLOW_SIM_GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(AlLowConstants.ALLOW_ARM_LENGTH_METERS, AlLowConstants.ALLOW_SIM_ARM_MASS_KG),
                AlLowConstants.ALLOW_ARM_LENGTH_METERS,
                Units.degreesToRadians(AlLowConstants.ALLOW_PIVOT_MIN_ANGLE),
                Units.degreesToRadians(AlLowConstants.ALLOW_PIVOT_MAX_ANGLE),
                // No gravity, and not a setting: SingleJointedArmSim measures
                // its angle from horizontal, while this arm's 0 deg is its
                // vertical stow, so the sim's gravity would act 90 deg out of
                // phase with the real arm and the sin(angle) feedforward.
                false,
                Units.degreesToRadians(AlLowConstants.ALLOW_PIVOT_MIN_ANGLE));
        }
    }

    /**
     * Builds and applies the pivot and roller configurations. Parameters are
     * persisted to flash so a brownout or power cycle cannot silently revert
     * the controllers to factory defaults mid-match.
     */
    private void configureMotors() {
        // --- Pivot configuration ---
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig
            .inverted(AlLowConstants.ALLOW_PIVOT_MOTOR_INVERTED)
            .idleMode(AlLowConstants.ALLOW_PIVOT_IDLE_MODE)
            .smartCurrentLimit(AlLowConstants.ALLOW_PIVOT_CURRENT_LIMIT)
            .voltageCompensation(AlLowConstants.ALLOW_NOMINAL_VOLTAGE);

        // Scale the NEO encoder so position is in degrees and velocity is in
        // degrees per second (native units are motor rotations and RPM).
        pivotConfig.encoder
            .positionConversionFactor(AlLowConstants.ALLOW_PIVOT_POSITION_CONVERSION)
            .velocityConversionFactor(AlLowConstants.ALLOW_PIVOT_VELOCITY_CONVERSION);

        // Closed-loop PID gains (slot 0). Error units are degrees after the
        // conversion factors above. Gravity compensation is not configured
        // here - it is angle-dependent, so it is passed per-cycle as
        // arbitrary feedforward in setSetpoint().
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(AlLowConstants.ALLOW_PIVOT_kP)
            .i(AlLowConstants.ALLOW_PIVOT_kI)
            .d(AlLowConstants.ALLOW_PIVOT_kD)
            .outputRange(AlLowConstants.ALLOW_PIVOT_MIN_OUTPUT, AlLowConstants.ALLOW_PIVOT_MAX_OUTPUT);

        // Soft limits (degrees) bound travel in every control mode
        pivotConfig.softLimit
            .forwardSoftLimit(AlLowConstants.ALLOW_PIVOT_MAX_ANGLE)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(AlLowConstants.ALLOW_PIVOT_MIN_ANGLE)
            .reverseSoftLimitEnabled(true);

        // --- Roller configuration ---
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        rollerConfig
            .inverted(AlLowConstants.ALLOW_ROLLER_MOTOR_INVERTED)
            .idleMode(AlLowConstants.ALLOW_ROLLER_IDLE_MODE)
            .smartCurrentLimit(AlLowConstants.ALLOW_ROLLER_CURRENT_LIMIT);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Gravity compensation voltage for the given arm angle. The arm is
     * vertical when stowed (0 degrees), so gravity torque scales with
     * sin(angle): zero stowed, maximal with the arm horizontal.
     */
    private double gravityFeedforward(double angleDegrees) {
        return AlLowConstants.ALLOW_PIVOT_kG * Math.sin(Math.toRadians(angleDegrees));
    }

    // Sends the latched target to the controller with gravity compensation
    // evaluated at the arm's measured angle.
    private void applyPositionReference() {
        pivotController.setSetpoint(
            targetAngle,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            gravityFeedforward(getPivotAngle()),
            ArbFFUnits.kVoltage);
    }

    /**
     * Commands the pivot to an angle in degrees under closed-loop control.
     * The Spark MAX holds the position afterward regardless of what commands
     * run; periodic() keeps the gravity feedforward tracking the arm.
     */
    public void setPivotAngle(double angle) {
        targetAngle = Math.min(Math.max(angle, AlLowConstants.ALLOW_PIVOT_MIN_ANGLE),
            AlLowConstants.ALLOW_PIVOT_MAX_ANGLE);

        positionControlEnabled = true;
        manualModeEnabled = false;

        applyPositionReference();
    }

    /**
     * Holds the current angle under closed-loop control. For an arm at rest
     * (the robot has just enabled); a moving arm is held with
     * {@link #holdWhereTheArmStops()}.
     */
    public void holdCurrentAngle() {
        setPivotAngle(getPivotAngle());
    }

    /**
     * Holds the angle a moving arm can stop at: the measured angle plus the
     * distance it covers while decelerating to rest over
     * ALLOW_MANUAL_RELEASE_STOP_SECONDS (velocity x time / 2). Targeting the
     * measured angle itself would put the target behind the arm the moment
     * it is set - the arm coasts on, and the loop drags it back: a bob on
     * every stick release. setPivotAngle clamps the result to the travel
     * limits.
     */
    public void holdWhereTheArmStops() {
        double stoppingDistance = getPivotVelocity() * AlLowConstants.ALLOW_MANUAL_RELEASE_STOP_SECONDS / 2.0;
        setPivotAngle(getPivotAngle() + stoppingDistance);
    }

    /**
     * Direct duty-cycle control from the operator stick. The subsystem
     * applies the deadband and speed limit, and holds the arm (where it can
     * stop) the moment the stick returns to center. Soft limits on the
     * controller stop travel at either end of the arm's range.
     */
    public void manualPivotControl(double stickInput) {
        if (Math.abs(stickInput) >= AlLowConstants.ALLOW_MANUAL_CONTROL_DEADBAND) {
            positionControlEnabled = false;
            manualModeEnabled = true;

            double speed = stickInput * AlLowConstants.ALLOW_MANUAL_SPEED_LIMIT;
            pivotMotor.set(Math.min(Math.max(speed, -1), 1)); // -1 to 1: the whole duty-cycle range, not a setting
        } else if (manualModeEnabled) {
            // Stick released this loop - hold where the still-moving arm can stop
            manualModeEnabled = false;
            holdWhereTheArmStops();
        }
        // Otherwise: position control (if active) keeps holding on the motor
        // controller; nothing to do.
    }

    /** Cuts pivot output and drops any closed-loop target (used when disabling). */
    public void stopPivot() {
        positionControlEnabled = false;
        manualModeEnabled = false;
        pivotMotor.set(0);
    }

    /**
     * Zeros the pivot encoder; only do this with the arm at its stowed
     * position. Drops any active closed-loop target first so the arm does
     * not lunge toward a now-meaningless setpoint.
     */
    public void resetPivotEncoder() {
        stopPivot();
        pivotEncoder.setPosition(0);
        targetAngle = 0.0;
    }

    /** Runs the intake rollers at the given duty cycle (-1 to 1). */
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    /** Stops the intake rollers. */
    public void stopRoller() {
        rollerMotor.set(0);
    }

    // ------------------------------------------------------------------
    // State getters (used by commands and the Dashboard)
    // ------------------------------------------------------------------

    /** Current arm angle in degrees (0 = stowed). */
    public double getPivotAngle() {
        return pivotEncoder.getPosition();
    }

    /** Current arm velocity in degrees per second (+ = deploying). */
    public double getPivotVelocity() {
        return pivotEncoder.getVelocity();
    }

    /** The angle in degrees the closed loop is targeting. */
    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtTargetAngle() {
        return Math.abs(targetAngle - getPivotAngle()) <= AlLowConstants.ALLOW_PIVOT_ALLOWED_ERROR;
    }

    public boolean isInManualMode() {
        return manualModeEnabled;
    }

    /** True while the closed loop is holding (or moving to) the target angle; false when the pivot output is cut or under manual control. */
    public boolean isHoldingPosition() {
        return positionControlEnabled;
    }

    /** Pivot motor output current in amps, for diagnostics. */
    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    /** Roller motor output current in amps, for diagnostics. */
    public double getRollerCurrent() {
        return rollerMotor.getOutputCurrent();
    }

    /** Pivot motor applied duty cycle, -1 to 1. */
    public double getPivotOutput() {
        return pivotMotor.getAppliedOutput();
    }

    /** Roller motor applied duty cycle, -1 to 1. */
    public double getRollerOutput() {
        return rollerMotor.getAppliedOutput();
    }

    @Override
    public void periodic() {
        // While holding a position, keep the gravity feedforward evaluated at
        // the arm's measured angle so the hold stays honest through sag and
        // disturbances. The reference itself is already latched on the
        // controller; this only refreshes the feedforward term.
        if (positionControlEnabled) {
            applyPositionReference();
        }
    }

    /**
     * Physics simulation: the Spark MAX sim runs the same closed-loop
     * controller as the real hardware, its applied output drives the arm
     * plant, and the resulting motion is fed back into the simulated
     * encoder (in degrees, matching the conversion factors).
     */
    @Override
    public void simulationPeriodic() {
        // 0.02 s is one robot loop (this runs once per loop), not a setting
        armSim.setInputVoltage(pivotMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        armSim.update(0.02);

        pivotMotorSim.iterate(
            Units.radiansToDegrees(armSim.getVelocityRadPerSec()),
            RobotController.getBatteryVoltage(),
            0.02);
    }
}
