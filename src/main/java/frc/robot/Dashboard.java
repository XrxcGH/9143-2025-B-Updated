package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AlLow;
import frc.robot.subsystems.KitBot;
import frc.robot.subsystems.Swerve;

/**
 * Central dashboard manager - the ONLY place in the robot code that
 * publishes dashboard data. Subsystems expose plain getters and know
 * nothing about the dashboard; update() polls them once per loop from
 * Robot.robotPeriodic().
 *
 * Everything is published as plain NetworkTables data under
 * /SmartDashboard, which both Elastic and AdvantageScope read directly
 * (Elastic dropped Shuffleboard API support, so no Shuffleboard calls are
 * used anywhere in this project). Persistent problems surface through
 * WPILib Alerts (Elastic's Alerts widget).
 */
public class Dashboard {
    private final Swerve swerve;
    private final KitBot kitbot;
    private final AlLow allow;

    /** Field widget data: robot pose (and any objects added later, e.g. trajectories). */
    private final Field2d field = new Field2d();

    /** Raw NT table backing Elastic's SwerveDrive widget (needs a ".type" marker). */
    private final NetworkTable swerveWidgetTable;

    // Cached entries for the SwerveDrive widget (FL angle, FL vel, FR angle,
    // FR vel, BL angle, BL vel, BR angle, BR vel, robot angle) - resolving
    // string-keyed entries on every 20 ms loop is pure waste.
    private final NetworkTableEntry[] swerveWidgetEntries;

    // Precomputed "Vision/<name> Has Target" keys (avoids per-loop concatenation)
    private final String[] visionHasTargetKeys;

    // ------------------------------------------------------------------
    // AlLow visualization
    // ------------------------------------------------------------------
    // Mechanism2d: side-on schematic of the AlLow arm, rendered by Glass
    // and AdvantageScope (both live over NT). Drawing convention: 0 deg
    // (stowed) points straight up; positive angles swing the arm outward.
    private final Mechanism2d allowMech = new Mechanism2d(1.2, 1.2);
    private final MechanismLigament2d allowArmLigament;

    // Approximate arm length for both visualizations (meters). VERIFY.
    private static final double ARM_LENGTH = 0.35;

    // 3D component pose for AdvantageScope's 3D field view: attach a glTF
    // CAD model and map this entry to the arm component in the 3D config.
    // Robot-relative frame: X forward, Y left, Z up, origin at the robot
    // center on the floor. Offsets are PLACEHOLDERS - VERIFY against the
    // CAD model's component origin.
    private static final double PIVOT_X_OFFSET = 0.0;  // Meters forward of robot center - VERIFY
    private static final double PIVOT_HEIGHT = 0.25;   // Pivot height above the floor (meters) - VERIFY
    private final Pose3d[] componentPoses = new Pose3d[1];

    /** Resting-voltage alert; only evaluated while disabled (sag under load is normal). */
    private final Alert lowBatteryAlert = new Alert(
        "Battery resting voltage is low - swap the battery before the next match.",
        AlertType.kWarning);

    /**
     * Registers all sendables. Call once from RobotContainer after the
     * subsystems exist. (The auto chooser is a LoggedDashboardChooser that
     * publishes itself - see RobotContainer.)
     */
    public Dashboard(Swerve swerve, KitBot kitbot, AlLow allow) {
        this.swerve = swerve;
        this.kitbot = kitbot;
        this.allow = allow;

        // --- AlLow arm Mechanism2d (Glass / AdvantageScope) ---
        allowArmLigament = allowMech.getRoot("AlLow", 0.6, 0.2)
            .append(new MechanismLigament2d("Arm", ARM_LENGTH, 90, 6,
                new Color8Bit(Color.kCyan)));

        // --- Sendables (registered once; NT keeps them updated) ---
        // Field widget: realtime robot location on the field drawing
        SmartDashboard.putData("Field", field);
        // AlLow arm schematic (viewable in Glass and AdvantageScope)
        SmartDashboard.putData("AlLow Mechanism", allowMech);
        // Command scheduler view for diagnostics
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        // Subsystem widgets (show default/current command) for diagnostics
        SmartDashboard.putData("AlLow Subsystem", allow);
        SmartDashboard.putData("KitBot Subsystem", kitbot);

        // --- Pre-match utility button (Command widget) ---
        // ignoringDisable lets the pit crew zero the arm without enabling.
        SmartDashboard.putData("Zero AlLow Pivot",
            Commands.runOnce(allow::resetPivotEncoder, allow)
                .ignoringDisable(true).withName("Zero AlLow Pivot"));

        // --- Elastic SwerveDrive widget ---
        // Elastic identifies the widget by a ".type" marker and reads the
        // module entries published in update() (angles in radians, m/s).
        swerveWidgetTable = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getSubTable("Swerve Drive");
        swerveWidgetTable.getEntry(".type").setString("SwerveDrive");
        swerveWidgetEntries = new NetworkTableEntry[] {
            swerveWidgetTable.getEntry("Front Left Angle"),
            swerveWidgetTable.getEntry("Front Left Velocity"),
            swerveWidgetTable.getEntry("Front Right Angle"),
            swerveWidgetTable.getEntry("Front Right Velocity"),
            swerveWidgetTable.getEntry("Back Left Angle"),
            swerveWidgetTable.getEntry("Back Left Velocity"),
            swerveWidgetTable.getEntry("Back Right Angle"),
            swerveWidgetTable.getEntry("Back Right Velocity"),
            swerveWidgetTable.getEntry("Robot Angle"),
        };

        visionHasTargetKeys = new String[VisionConstants.LIMELIGHT_NAMES.length];
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            visionHasTargetKeys[i] = "Vision/" + VisionConstants.LIMELIGHT_NAMES[i] + " Has Target";
        }
    }

    /**
     * Publishes all live values. Called every loop from Robot.robotPeriodic().
     */
    public void update() {
        // --- Field + drivetrain ---
        var driveState = swerve.getState();
        field.setRobotPose(driveState.Pose);

        SmartDashboard.putNumber("Swerve/Speed",
            Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond));
        SmartDashboard.putNumber("Swerve/Heading", driveState.Pose.getRotation().getDegrees());
        SmartDashboard.putBoolean("Swerve/Vision Tracking", swerve.isVisionTrackingEnabled());

        // SwerveDrive widget entries (module order: FL, FR, BL, BR; entries
        // cached in the constructor)
        if (driveState.ModuleStates != null && driveState.ModuleStates.length == 4) {
            for (int i = 0; i < 4; i++) {
                swerveWidgetEntries[i * 2].setDouble(driveState.ModuleStates[i].angle.getRadians());
                swerveWidgetEntries[i * 2 + 1].setDouble(driveState.ModuleStates[i].speedMetersPerSecond);
            }
            swerveWidgetEntries[8].setDouble(driveState.Pose.getRotation().getRadians());
        }

        // --- AlLow visualization ---
        // Mechanism2d convention: 0 deg (stowed) draws straight up, deploy
        // angles swing outward.
        double armAngleDeg = allow.getPivotAngle();
        allowArmLigament.setAngle(90.0 - armAngleDeg);

        // 3D component pose for AdvantageScope (robot-relative: X forward,
        // Y left, Z up). Arm pitches about the Y axis; the sign/zero must
        // match the CAD component's modeled orientation - VERIFY in
        // AdvantageScope and flip/offset here if the model swings backward.
        componentPoses[0] = new Pose3d(PIVOT_X_OFFSET, 0, PIVOT_HEIGHT,
            new Rotation3d(0, Units.degreesToRadians(armAngleDeg), 0));

        // --- AdvantageKit structured outputs (.wpilog + RLOG live stream) ---
        // These are the review-critical fields for AdvantageScope: 2D/3D
        // field views, swerve visualization, and mechanism traces.
        Logger.recordOutput("RobotState/Pose", Pose2d.struct, driveState.Pose);
        Logger.recordOutput("RobotState/Speeds", ChassisSpeeds.struct, driveState.Speeds);
        if (driveState.ModuleStates != null && driveState.ModuleStates.length == 4) {
            Logger.recordOutput("RobotState/ModuleStates", SwerveModuleState.struct, driveState.ModuleStates);
            Logger.recordOutput("RobotState/ModuleTargets", SwerveModuleState.struct, driveState.ModuleTargets);
        }
        Logger.recordOutput("RobotState/ComponentPoses", Pose3d.struct, componentPoses);
        Logger.recordOutput("AlLow/AngleDegrees", armAngleDeg);
        Logger.recordOutput("AlLow/TargetDegrees", allow.getTargetAngle());
        Logger.recordOutput("KitBot/RollerOutput", kitbot.getRollerOutput());
        Logger.recordOutput("Vision/BestTag",
            swerve.getVision().getBestTarget().map(t -> t.id).orElse(-1));

        // --- Match / robot vitals ---
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CAN Utilization",
            RobotController.getCANStatus().percentBusUtilization);

        // --- AlLow ---
        SmartDashboard.putNumber("AlLow/Angle", allow.getPivotAngle());
        SmartDashboard.putNumber("AlLow/Target", allow.getTargetAngle());
        SmartDashboard.putBoolean("AlLow/At Target", allow.isAtTargetAngle());
        SmartDashboard.putBoolean("AlLow/Manual Mode", allow.isInManualMode());
        SmartDashboard.putNumber("AlLow/Pivot Current", allow.getPivotCurrent());
        SmartDashboard.putNumber("AlLow/Roller Current", allow.getRollerCurrent());
        SmartDashboard.putNumber("AlLow/Pivot Output", allow.getPivotOutput());
        SmartDashboard.putNumber("AlLow/Roller Output", allow.getRollerOutput());

        // --- KitBot ---
        SmartDashboard.putNumber("KitBot/Roller Current", kitbot.getRollerCurrent());
        SmartDashboard.putNumber("KitBot/Roller Output", kitbot.getRollerOutput());

        // --- Vision (inert until a Limelight is configured) ---
        var vision = swerve.getVision();
        SmartDashboard.putNumber("Vision/Best Tag",
            vision.getBestTarget().map(t -> (double) t.id).orElse(-1.0));
        SmartDashboard.putNumber("Vision/TX",
            vision.getBestTarget().map(t -> t.tx).orElse(0.0));
        SmartDashboard.putNumber("Vision/Distance",
            vision.getBestTarget().map(t -> t.groundDistance()).orElse(0.0));
        for (int i = 0; i < visionHasTargetKeys.length; i++) {
            SmartDashboard.putBoolean(visionHasTargetKeys[i], vision.hasTarget(i));
        }

        // --- Alerts (persistent conditions) ---
        // Resting-voltage check only while disabled - voltage sags under
        // load during a match are normal and would nag the drive team.
        lowBatteryAlert.set(DriverStation.isDisabled()
            && RobotController.getBatteryVoltage() < 12.0);
    }
}
