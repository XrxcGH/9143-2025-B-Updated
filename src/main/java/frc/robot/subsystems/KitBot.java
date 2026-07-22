package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.KitBotConstants;

/**
 * KitBot coral roller: a single NEO on a Spark MAX that ejects game pieces
 * into the reef's L1 trough (negative duty cycle ejects; positive pulls a
 * piece back in to re-seat it).
 *
 * All roller behavior is exposed as command factories, so button bindings
 * and PathPlanner NamedCommands both consume the same definitions and the
 * roller always stops when a command ends or is interrupted.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class KitBot extends SubsystemBase {

    private final SparkMax rollerMotor;

    public KitBot() {
        rollerMotor = new SparkMax(KitBotConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Because parameters are only set once on construction, the CAN
        // timeout can be long without blocking robot operation.
        rollerMotor.setCANTimeout(250);

        // Voltage compensation keeps the roller behaving the same as the
        // battery voltage dips; the current limit prevents breaker trips or
        // burning out the motor if the roller stalls. Persisted to flash so
        // a brownout cannot revert the controller to factory defaults.
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig
            .voltageCompensation(KitBotConstants.ROLLER_MOTOR_VOLTAGE_COMP)
            .smartCurrentLimit(KitBotConstants.ROLLER_MOTOR_CURRENT_LIMIT);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ------------------------------------------------------------------
    // Command factories (each requires this subsystem and stops the roller
    // when it ends or is interrupted)
    // ------------------------------------------------------------------

    /** Ejects the first (single) piece: timed run at the first-eject speed. */
    public Command ejectFirstPiece() {
        return startEnd(
                () -> rollerMotor.set(KitBotConstants.ROLLER_FIRST_EJECT_VALUE),
                rollerMotor::stopMotor)
            .withTimeout(KitBotConstants.ROLLER_EJECT_DURATION)
            .withName("EjectFirstPiece");
    }

    /** Ejects a stacked piece: timed run at the (stronger) stacked-eject speed. */
    public Command ejectStackedPiece() {
        return startEnd(
                () -> rollerMotor.set(KitBotConstants.ROLLER_STACKED_EJECT_VALUE),
                rollerMotor::stopMotor)
            .withTimeout(KitBotConstants.ROLLER_EJECT_DURATION)
            .withName("EjectStackedPiece");
    }

    /** Pulls a piece back in to re-seat it; runs until interrupted. */
    public Command realignPiece() {
        return startEnd(
                () -> rollerMotor.set(KitBotConstants.ROLLER_REALIGN_VALUE),
                rollerMotor::stopMotor)
            .withName("RealignPiece");
    }

    /** Creeps a piece forward for fine positioning; runs until interrupted. */
    public Command jogPiece() {
        return startEnd(
                () -> rollerMotor.set(KitBotConstants.ROLLER_JOG_VALUE),
                rollerMotor::stopMotor)
            .withName("JogPiece");
    }

    /** Stops the roller immediately (used when disabling). */
    public void stop() {
        rollerMotor.stopMotor();
    }

    // ------------------------------------------------------------------
    // State getters (used by the Dashboard)
    // ------------------------------------------------------------------

    /** Roller motor output current in amps, for diagnostics. */
    public double getRollerCurrent() {
        return rollerMotor.getOutputCurrent();
    }

    /** Roller motor applied duty cycle, -1 to 1. */
    public double getRollerOutput() {
        return rollerMotor.getAppliedOutput();
    }
}
