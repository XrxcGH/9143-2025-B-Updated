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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants.AlLowConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants.TagClass;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.AlLow;
import frc.robot.subsystems.Vision;
import frc.robot.util.Rumble;
import frc.robot.util.Tunables;

/**
 * RobotContainer owns every subsystem and maps controller inputs to commands.
 * This is the single place to look up "what does this button do".
 *
 * ================================ CONTROLS =================================
 * Anything that drives the robot by itself is a hold, never a toggle; anything
 * rare or dangerous is Test-mode only, or disabled-only behind a held button.
 * The numbers behind these bindings (speeds, preset angles, hold times,
 * trigger thresholds, ports) are in Constants.
 *
 * DRIVER (ControllerConstants.DRIVER_CONTROLLER_PORT):
 *   Left stick          - field-centric translation (scaled by the "Drive -
 *                         Teleop Speed Scale" tunable, default
 *                         DriveConstants.TELEOP_SPEED_SCALE)
 *   Right stick X       - rotation (same scale)
 *   A (hold)            - X-lock the wheels (brake)
 *   Left bumper         - driver heading zero: the way the robot faces now =
 *                         stick forward (back+LB also re-seeds the pose heading)
 *   D-pad               - slow robot-centric nudges, all 8 directions
 *   B, Back/Start + X/Y - point wheels / SysId: TEST MODE ONLY
 *
 *   The next four exist only once a Limelight is configured in
 *   VisionConstants (this robot currently has no camera, so they are unbound):
 *   Left trigger (hold) - align on the reef: bumper flush and centered on the
 *                         face in view, for the KitBot L1 eject; release =
 *                         sticks back instantly
 *   Right trigger (hold)- align on the coral station: bumper flush, centered
 *   Y                   - re-seed the pose heading from the AprilTags in view
 *                         (MegaTag1 fused for HEADING_RESEED_WINDOW_SECONDS);
 *                         the driver's own "forward" does not move
 *   Rumble (driver)     - steady while an alignment is held and aligned
 *
 * OPERATOR (ControllerConstants.OPERATOR_CONTROLLER_PORT):
 *   Right stick Y       - AlLow pivot manual control (holds angle on release)
 *   D-pad down          - AlLow deploy to the INTAKE preset angle, rollers in
 *   D-pad right         - AlLow to the HOLD preset angle, rollers stopped
 *   D-pad up            - AlLow stow (the BASE preset), rollers stopped
 *   Left trigger (hold) - AlLow rollers intake
 *   Right trigger (hold)- AlLow rollers eject
 *   B                   - KitBot eject first piece (timed)
 *   Y                   - KitBot eject stacked piece (timed)
 *   X (hold)            - KitBot re-align piece
 *   A (hold)            - KitBot jog piece
 *   Start, held ALLOW_ZERO_HOLD_SECONDS, DISABLED only - zero the AlLow
 *                         pivot encoder (arm at its stow)
 * ===========================================================================
 */
public class RobotContainer {
    /** Top speed from swerve characterization, used to scale driver input. */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    /** Rotation rate at full stick and a speed scale of 1.0. */
    private double MaxAngularRate =
        RotationsPerSecond.of(DriveConstants.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND).in(RadiansPerSecond);

    // ------------------------------------------------------------------
    // Reusable swerve requests for teleop driving (allocated once)
    // ------------------------------------------------------------------
    // Drive requests use closed-loop velocity, not open-loop voltage:
    // every module tracks the true requested ground speed regardless of
    // battery sag, and teleop behavior matches autonomous path following.
    /**
     * Standard field-centric drive. Speed scaling and the matching deadbands
     * are applied per loop in the default command (the scale is a dashboard
     * tunable), so nothing speed-dependent is baked in here.
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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
    private final CommandXboxController driver_controller =
        new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator_controller =
        new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    /** Haptic cue for the driver: aligned. */
    private final Rumble driverRumble = new Rumble(driver_controller);

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
    /** Raised when PathPlanner could not be configured (no autos will be offered). */
    private final Alert autoBuilderAlert = new Alert(
        "PathPlanner AutoBuilder not configured (deploy/pathplanner/settings.json missing or invalid): no autos available",
        Alert.AlertType.kError);

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
        // The default (AutoConstants.DEFAULT_AUTO_NAME) must name an auto that
        // actually exists there. LoggedDashboardChooser publishes it under
        // SmartDashboard/Auto Mode (Elastic's ComboBox Chooser widget; the key
        // is the layout's, so it stays here) and records the selection in
        // the AdvantageKit log.
        SendableChooser<Command> chooser;
        if (AutoBuilder.isConfigured()) {
            chooser = AutoBuilder.buildAutoChooser(AutoConstants.DEFAULT_AUTO_NAME);
        } else {
            // Swerve.configureAutoBuilder already reported why. buildAutoChooser
            // would throw here and take the whole robot program down with it.
            chooser = new SendableChooser<>();
            chooser.setDefaultOption("None (AutoBuilder not configured)", Commands.none());
            autoBuilderAlert.set(true);
        }
        autoChooser = new LoggedDashboardChooser<>("Auto Mode", chooser);

        // Second autonomous option: add every Choreo trajectory from
        // deploy/choreo to the same chooser (see addChoreoAutos).
        addChoreoAutos();

        // (Tunables.init() runs in Robot before this container is built.)

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
    // Driver bindings (drivetrain + align)
    // ==================================================================

    /**
     * Hold-to-align: selects what to align on (the KitBot has no piece
     * sensor, so the driver says which - reef or coral station), tracks
     * while held, and hands the sticks straight back on release (the
     * tracking command's own finallyDo stops the robot and clears the
     * tracking state). One new command per binding - a command instance
     * cannot sit in two compositions.
     */
    private Command alignTo(TagClass tagClass) {
        return Commands.runOnce(() -> {
                swerve.getVision().setAlignmentClass(tagClass);
                swerve.setVisionTrackingEnabled(true);
            })
            .andThen(swerve.createAprilTagTrackingCommand())
            .withName("Align " + tagClass);
    }

    private void configureSwerveBindings() {
        // Default command: field-centric driving from the sticks (closed-loop
        // velocity; see the drive request note above).
        // WPILib convention: X is forward and Y is to the left.
        swerve.setDefaultCommand(
            swerve.applyRequest(() -> {
                double scale = Tunables.teleopSpeedScale();
                double maxSpeed = MaxSpeed * scale;
                double maxAngularRate = MaxAngularRate * scale;
                return drive
                    .withDeadband(maxSpeed * DriveConstants.STICK_DEADBAND)
                    .withRotationalDeadband(maxAngularRate * DriveConstants.STICK_DEADBAND)
                    .withVelocityX(-driver_controller.getLeftY() * maxSpeed)  // Forward with negative Y (stick up)
                    .withVelocityY(-driver_controller.getLeftX() * maxSpeed)  // Left with negative X
                    .withRotationalRate(-driver_controller.getRightX() * maxAngularRate); // CCW with negative X (stick left)
            })
        );

        Trigger testMode = new Trigger(DriverStation::isTest);

        // A: X-lock. B (point wheels) and the SysId chords only exist in Test
        // mode: SysId applies open-loop voltage steps to the drivetrain, and
        // neither belongs under a thumb during a match. Each SysId routine
        // should be run exactly once in a single log.
        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().and(testMode).whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))));
        driver_controller.back().and(driver_controller.y()).and(testMode).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).and(testMode).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).and(testMode).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).and(testMode).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // D-pad nudges in all eight directions from the POV angle, so a thumb
        // that lands on a diagonal still moves the robot (povUp() and the
        // other cardinal triggers are true only at exactly their own angle,
        // so bindings on those alone would ignore a 45-degree press).
        new Trigger(() -> driver_controller.getHID().getPOV() >= 0).whileTrue(swerve.applyRequest(() -> {
            double pov = Math.toRadians(driver_controller.getHID().getPOV()); // 0 = up, clockwise
            double speed = DriveConstants.NUDGE_SPEED_METERS_PER_SECOND;
            return forwardStraight.withVelocityX(speed * Math.cos(pov)).withVelocityY(-speed * Math.sin(pov));
        }));

        // Driver heading zero on left bumper: "the way the robot faces now is
        // forward on my stick". It only moves the driver's frame (held in the
        // raw gyro frame - see Swerve.periodic), never the pose estimator's
        // heading, so it is safe at any time. While disabled with no tag
        // supplying a heading (always the case with no camera mounted) it
        // also seeds the pose heading to the alliance's forward direction;
        // back + left bumper forces that seed.
        driver_controller.leftBumper().and(driver_controller.back().negate())
            .onTrue(Commands.runOnce(() -> swerve.zeroDriverHeading(
                DriverStation.isDisabled() && !swerve.getVision().hasFreshHeadingSeed())).ignoringDisable(true));
        driver_controller.back().and(driver_controller.leftBumper())
            .onTrue(Commands.runOnce(() -> swerve.zeroDriverHeading(true)).ignoringDisable(true));

        // Everything that needs a camera is bound only when one is configured.
        // With LIMELIGHT_NAMES empty the tracking command would find no target
        // and hold the drivetrain stopped for as long as a trigger was
        // held, so an accidental press must not be able to do that.
        if (Vision.hasCameras()) {
            // Hold a trigger to align; release = sticks. Never a toggle: a
            // robot that keeps driving itself after the driver has let go is
            // the failure to avoid.
            driver_controller.leftTrigger(DriveConstants.ALIGN_TRIGGER_THRESHOLD)
                .whileTrue(alignTo(TagClass.REEF));
            driver_controller.rightTrigger(DriveConstants.ALIGN_TRIGGER_THRESHOLD)
                .whileTrue(alignTo(TagClass.CORAL_STATION));

            // Y: correct the pose heading from tag geometry. While enabled only
            // MegaTag2 is fused, and MegaTag2 takes its heading from the pose, so
            // a heading that has drifted (a hard hit, a long match) is never
            // corrected by it; this fuses MegaTag1, whose solve carries its own
            // heading, for HEADING_RESEED_WINDOW_SECONDS. With no tag in view it
            // does nothing. The driver's frame is held in the raw gyro frame, so
            // "forward" on the stick does not move. (Not in Test mode, where
            // Back / Start + Y are the SysId bindings.)
            driver_controller.y().and(testMode.negate())
                .onTrue(Commands.runOnce(() -> swerve.getVision().requestHeadingReseed()).ignoringDisable(true));

            // Aligned: driver, steady light buzz -> call for the eject.
            new Trigger(() -> swerve.isVisionTrackingEnabled() && swerve.isAligned())
                .whileTrue(driverRumble.whileActive(DriveConstants.ALIGNED_RUMBLE_STRENGTH));
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
        operator_controller.rightTrigger(ControllerConstants.OPERATOR_TRIGGER_THRESHOLD).whileTrue(Commands.startEnd(
            () -> allow.setRollerSpeed(AlLowConstants.ALLOW_ROLLER_EJECT_SPEED),
            allow::stopRoller, allow));
        operator_controller.leftTrigger(ControllerConstants.OPERATOR_TRIGGER_THRESHOLD).whileTrue(Commands.startEnd(
            () -> allow.setRollerSpeed(AlLowConstants.ALLOW_ROLLER_INTAKE_SPEED),
            allow::stopRoller, allow));

        // Encoder zeroing: only while disabled, only after a hold of
        // ALLOW_ZERO_HOLD_SECONDS, with the arm at its stowed position.
        // Zeroing a deployed arm mid-match would silently shift the soft
        // limits and every preset by the arm's current angle
        // (resetPivotEncoder drops the closed loop first, so there is no
        // lunge - but the corrupted reference frame remains).
        operator_controller.start().and(DriverStation::isDisabled)
            .debounce(AlLowConstants.ALLOW_ZERO_HOLD_SECONDS)
            .onTrue(Commands.runOnce(allow::resetPivotEncoder, allow).ignoringDisable(true));

        // -------- Manual override (default command) --------
        // The subsystem applies the deadband and speed limit, and holds the
        // arm under closed loop when the stick is released - a centered
        // stick never fights an active position hold.
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
     * PathPlanner auto or a Choreo trajectory from the same dropdown. Draw a
     * trajectory in the Choreo app (saving into src/main/deploy/choreo) and it
     * appears here automatically - exactly how PathPlanner autos are picked up
     * from deploy/pathplanner/autos.
     *
     * The trajectory is followed by the same PathPlanner AutoBuilder holonomic
     * controller and AutoConstants gains as the PathPlanner autos: PathPlanner
     * 2026 natively loads Choreo .traj files ({@code fromChoreoTrajectory}),
     * and ChoreoLib has no 2026 release, so this keeps a single, already-tuned
     * path-following code path. A malformed or unreadable file is reported and
     * skipped rather than crashing robot construction.
     */
    private void addChoreoAutos() {
        // The folder PathPlanner's Choreo loader reads from, not a setting
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
     * the trajectory's starting pose (keeping a fresh vision heading seed -
     * see Swerve.resetPoseForAuto), then follow it. The start pose is
     * alliance-flipped to match how AutoBuilder mirrors the path itself on the
     * red alliance, so a Choreo auto lines up correctly on both alliances.
     */
    private Command choreoAutoCommand(PathPlannerPath path) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                Pose2d start = path.getStartingHolonomicPose().orElse(swerve.getState().Pose);
                swerve.resetPoseForAuto(AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(start) : start);
            }),
            AutoBuilder.followPath(path)
        );
    }

    /**
     * Called from Robot.disabledExit(): hold the AlLow arm where it is.
     * Disabling cuts its output and drops the closed loop, and nothing
     * commands it again until the operator presses something. Without this
     * hold, an arm that is deployed when the robot enables (after an auto
     * that ended with it down, say) hangs on the motor's brake mode alone
     * and sags. The arm is at rest here, so its measured angle is where it
     * can stop. The KitBot roller has nothing to hold.
     */
    public void enabledInit() {
        allow.holdCurrentAngle();
    }

    /** Called from Robot.disabledInit(): stop every mechanism output. */
    public void disabledInit() {
        kitbot.stop();
        allow.stopPivot();
        allow.stopRoller();
    }
}
