package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import java.io.File;
import java.util.Arrays;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.AlLowConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.AlLow;

/**
 * RobotContainer owns every subsystem and maps controller inputs to commands.
 * This is the single place to look up "what does this button do".
 *
 * ================================ CONTROLS =================================
 * DRIVER (port 0):
 *   Left stick          - field-centric translation (forward/strafe)
 *   Right stick X       - rotation
 *   A (hold)            - X-lock the wheels (brake)
 *   B (hold)            - point all modules at the left-stick direction
 *   Y (press)           - toggle AprilTag vision tracking (only bound once
 *                         a Limelight is configured in VisionConstants -
 *                         this robot currently has no cameras)
 *   D-pad               - slow robot-centric nudges (up/down/left/right)
 *   Left bumper         - re-zero field-centric heading
 *   Back/Start + X/Y    - SysId characterization routines (test setup only)
 *
 * OPERATOR (port 1):
 *   Right stick Y       - AlLow pivot manual control (holds angle on release)
 *   D-pad down          - AlLow deploy to intake angle (45 deg), rollers in
 *   D-pad right         - AlLow to hold angle (15 deg), rollers stopped
 *   D-pad up            - AlLow stow (0 deg), rollers stopped
 *   Left trigger (hold) - AlLow rollers intake
 *   Right trigger (hold)- AlLow rollers eject
 *   B                   - KitBot eject first piece (timed)
 *   Y                   - KitBot eject stacked piece (timed)
 *   X (hold)            - KitBot re-align piece
 *   A (hold)            - KitBot jog piece
 *   Start               - reset AlLow pivot encoder (works while disabled)
 * ===========================================================================
 */
public class RobotContainer {
    /** Top speed from swerve characterization, used to scale driver input. */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Max rotation rate for driver input: 3/4 rotation per second. */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // ------------------------------------------------------------------
    // Reusable swerve requests for teleop driving (allocated once)
    // ------------------------------------------------------------------
    // NOTE: drive requests use CLOSED-LOOP velocity, not open-loop voltage:
    // every module tracks the true requested ground speed regardless of
    // battery sag, and teleop behavior matches autonomous path following.
    /** Standard field-centric drive with a 20% stick deadband. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2)
        .withDriveRequestType(DriveRequestType.Velocity);
    /** X-locks the wheels to resist being pushed. */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    /** Points all modules in a direction without driving (alignment/testing). */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    /** Robot-centric drive used for the slow D-pad nudges. */
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);

    /** Publishes swerve state to NetworkTables/SignalLogger every odometry update. */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers: driver handles the drivetrain, operator handles mechanisms
    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    // ------------------------------------------------------------------
    // Subsystems
    // ------------------------------------------------------------------
    // Swerve creates its own Vision instance internally; constructing a
    // second one elsewhere would double up pose updates.
    public final Swerve swerve = TunerConstants.createDrivetrain();
    private final KitBot kitbot = new KitBot();
    private final AlLow allow = new AlLow();

    /** Dashboard chooser for selecting the autonomous routine (logged through
     *  AdvantageKit so every log records which auto was selected). */
    private final LoggedDashboardChooser<Command> autoChooser;

    /** Central dashboard publisher; updated from Robot.robotPeriodic(). */
    private final Dashboard dashboard;

    public RobotContainer() {
        // Register the commands PathPlanner autos reference by name. The
        // names are part of the .auto files - do not rename one side only.
        NamedCommands.registerCommand("EjectFirstPieceCommand", kitbot.ejectFirstPiece());
        NamedCommands.registerCommand("EjectStackedPieceCommand", kitbot.ejectStackedPiece());
        NamedCommands.registerCommand("RealignPieceCommand", kitbot.realignPiece());
        NamedCommands.registerCommand("JogPieceCommand", kitbot.jogPiece());

        // Auto chooser is populated with every auto in deploy/pathplanner/autos.
        // The default must name an auto that actually exists there.
        // LoggedDashboardChooser publishes it under SmartDashboard/Auto Mode
        // (Elastic's ComboBox Chooser widget) AND records the selection in
        // the AdvantageKit log.
        autoChooser = new LoggedDashboardChooser<>("Auto Mode",
            AutoBuilder.buildAutoChooser("Center Drop"));

        // Second autonomous option: add every Choreo trajectory from
        // deploy/choreo to the same chooser (see addChoreoAutos).
        addChoreoAutos();

        // All dashboard/NetworkTables publishing is centralized here.
        dashboard = new Dashboard(swerve, kitbot, allow);

        configureBindings();
    }

    /** Publishes all dashboard data; called every loop by Robot.robotPeriodic(). */
    public void updateDashboard() {
        dashboard.update();
    }

    private void configureBindings() {
        configureSwerveBindings();
        configureMechanismBindings();
    }

    // ==================================================================
    // Driver bindings (drivetrain)
    // ==================================================================
    private void configureSwerveBindings() {
        // Default command: field-centric driving from the sticks (closed-loop
        // velocity; see the drive request note above).
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Translation is scaled to 75%; rotation uses the full rate.
        swerve.setDefaultCommand(
            swerve.applyRequest(() ->
                drive.withVelocityX(-driver_controller.getLeftY() * MaxSpeed * 0.75) // Forward with negative Y (stick up)
                    .withVelocityY(-driver_controller.getLeftX() * MaxSpeed * 0.75)  // Left with negative X
                    .withRotationalRate(-driver_controller.getRightX() * MaxAngularRate) // CCW with negative X (stick left)
            )
        );

        // A: X-lock wheels; B: point modules at the left-stick direction
        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))
        ));

        // D-pad: slow robot-centric nudges for lining up on field elements
        driver_controller.povUp().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver_controller.povDown().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driver_controller.povLeft().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        driver_controller.povRight().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_controller.back().and(driver_controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driver_controller.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        // Toggle vision tracking on Y, but not while back/start are held
        // (back+Y and start+Y are the SysId test combos above). The toggle
        // keys off whether the tracking command is actually SCHEDULED, not a
        // parallel flag - the command's own finallyDo stops the robot and
        // clears the tracking state whenever it ends, including when another
        // swerve binding (brake, point, nudges, SysId) interrupts it, so the
        // toggle can never desync from reality. Only bound when at least one
        // Limelight is configured: with no cameras the tracking command
        // would just freeze the drivetrain until toggled off, so an
        // accidental press must not be able to do that.
        if (VisionConstants.LIMELIGHT_NAMES.length > 0) {
            driver_controller.y()
                .and(driver_controller.back().negate())
                .and(driver_controller.start().negate())
                .onTrue(Commands.runOnce(() -> {
                if (swerve.aprilTagTrackingCommand.isScheduled()) {
                    swerve.aprilTagTrackingCommand.cancel();
                } else {
                    swerve.setVisionTrackingEnabled(true);
                    CommandScheduler.getInstance().schedule(swerve.aprilTagTrackingCommand);
                }
            }));
        }

        // Stream drivetrain state to NetworkTables + SignalLogger for analysis
        swerve.registerTelemetry(logger::telemeterize);
    }

    // ==================================================================
    // Operator bindings (KitBot roller + AlLow intake)
    // ==================================================================
    private void configureMechanismBindings() {
        // -------- KitBot roller --------
        operator_controller.b().onTrue(kitbot.ejectFirstPiece());
        operator_controller.y().onTrue(kitbot.ejectStackedPiece());
        operator_controller.x().whileTrue(kitbot.realignPiece());
        operator_controller.a().whileTrue(kitbot.jogPiece());

        // -------- AlLow presets --------
        // Each preset fires once; the Spark MAX latches the angle reference
        // and holds it from there, so no repeating command is needed.

        // Deploy to the intake angle with the rollers pulling in
        operator_controller.povDown().onTrue(Commands.runOnce(() -> {
            allow.setPivotAngle(AlLowConstants.PivotPresetAngles.INTAKE.getAngle());
            allow.setRollerSpeed(AlLowConstants.ALLOW_ROLLER_INTAKE_SPEED);
        }, allow));

        // Raise to the hold angle (piece clear of the ground), rollers stopped
        operator_controller.povRight().onTrue(Commands.runOnce(() -> {
            allow.setPivotAngle(AlLowConstants.PivotPresetAngles.HOLD.getAngle());
            allow.stopRoller();
        }, allow));

        // Stow, rollers stopped
        operator_controller.povUp().onTrue(Commands.runOnce(() -> {
            allow.setPivotAngle(AlLowConstants.PivotPresetAngles.BASE.getAngle());
            allow.stopRoller();
        }, allow));

        // -------- AlLow rollers (hold to run; stop on release) --------
        // These require the subsystem, which pauses the manual default
        // command while held - the pivot keeps holding its angle on the
        // controller throughout.
        operator_controller.rightTrigger().whileTrue(Commands.startEnd(
            () -> allow.setRollerSpeed(AlLowConstants.ALLOW_ROLLER_EJECT_SPEED),
            allow::stopRoller, allow));
        operator_controller.leftTrigger().whileTrue(Commands.startEnd(
            () -> allow.setRollerSpeed(AlLowConstants.ALLOW_ROLLER_INTAKE_SPEED),
            allow::stopRoller, allow));

        // Encoder reset: ONLY while disabled, with the arm at its stowed
        // position. Zeroing a deployed arm mid-match would silently shift
        // the soft limits and every preset by the arm's current angle
        // (resetPivotEncoder drops the closed loop first, so there is no
        // lunge - but the corrupted reference frame remains).
        operator_controller.start().and(DriverStation::isDisabled).onTrue(
            Commands.runOnce(allow::resetPivotEncoder, allow).ignoringDisable(true));

        // -------- Manual override (default command) --------
        // The subsystem applies the deadband and speed limit, and holds the
        // current angle under closed loop when the stick is released - a
        // centered stick never fights an active position hold.
        allow.setDefaultCommand(Commands.run(() ->
            allow.manualPivotControl(-operator_controller.getRightY()), allow));
    }

    /** Returns the autonomous routine selected on the dashboard. */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    // ==================================================================
    // Choreo autonomous option (alongside PathPlanner)
    // ==================================================================
    /**
     * Discovers every Choreo trajectory (.traj) in deploy/choreo and adds it
     * to the auto chooser as "Choreo: &lt;name&gt;", so the driver can pick a
     * PathPlanner auto OR a Choreo trajectory from the same dropdown. Draw a
     * trajectory in the Choreo app (saving into src/main/deploy/choreo) and it
     * appears here automatically - exactly how PathPlanner autos are picked up
     * from deploy/pathplanner/autos.
     *
     * The trajectory is followed by the SAME PathPlanner AutoBuilder holonomic
     * controller and AutoConstants gains as the PathPlanner autos: PathPlanner
     * 2026 natively loads Choreo .traj files ({@code fromChoreoTrajectory}),
     * and ChoreoLib has no 2026 release, so this keeps a single, already-tuned
     * path-following code path. A malformed or unreadable file is reported and
     * skipped rather than crashing robot construction.
     */
    private void addChoreoAutos() {
        File choreoDir = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] trajFiles = choreoDir.listFiles((dir, name) -> name.endsWith(".traj"));
        if (trajFiles == null) {
            return; // no deploy/choreo directory present
        }
        Arrays.sort(trajFiles);
        for (File file : trajFiles) {
            String name = file.getName().substring(0, file.getName().length() - ".traj".length());
            try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
                autoChooser.addOption("Choreo: " + name, choreoAutoCommand(path));
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load Choreo trajectory '" + name + "'", ex.getStackTrace());
            }
        }
    }

    /**
     * Builds a standalone auto from a Choreo-sourced path: reset odometry to
     * the trajectory's starting pose, then follow it. The start pose is
     * alliance-flipped to match how AutoBuilder mirrors the path itself on the
     * red alliance, so a Choreo auto lines up correctly on both alliances.
     */
    private Command choreoAutoCommand(PathPlannerPath path) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                Pose2d start = path.getStartingHolonomicPose().orElse(swerve.getState().Pose);
                swerve.resetPose(AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(start) : start);
            }),
            AutoBuilder.followPath(path)
        );
    }

    /** Called from Robot.disabledInit(): stop every mechanism output. */
    public void disabledInit() {
        kitbot.stop();
        allow.stopPivot();
        allow.stopRoller();
    }
}
