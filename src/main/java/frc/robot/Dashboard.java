package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
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
import frc.robot.util.Tunables;

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
 * WPILib Alerts (Elastic's Alerts widget). Tunable "magic numbers" are
 * edited on the Testing tab through WPILib Preferences (see Tunables).
 *
 * The Vision/ topics are published whether or not a Limelight is
 * configured, so the layout's widgets always have a source; with no camera
 * they simply read "nothing seen" (-1, "", false).
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
    /** Last pose fused from each Limelight, drawn on the Field widget. */
    private final FieldObject2d[] visionFieldObjects;

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
        // ignoringDisable lets the pit crew zero the arm without enabling -
        // and ONLY without enabling: zeroing a deployed arm shifts the soft
        // limits and every preset by its current angle, so the button does
        // nothing while the robot is enabled (the same rule as the
        // operator's Start zeroing binding). The check lives inside a
        // command with NO requirement, so a click while enabled is fully
        // inert - requiring the subsystem would interrupt whatever AlLow
        // command is running (e.g. held rollers). While disabled nothing
        // else can be running on the subsystem, so none is needed.
        SmartDashboard.putData("Zero AlLow Pivot",
            Commands.runOnce(() -> {
                if (DriverStation.isDisabled()) {
                    allow.resetPivotEncoder();
                }
            }).ignoringDisable(true).withName("Zero AlLow Pivot"));

        // Restores every dashboard-tunable value (Robot Preferences widget)
        // to its Constants default; usable while disabled.
        SmartDashboard.putData("Testing/Reset Tunables",
            Commands.runOnce(Tunables::resetToDefaults)
                .ignoringDisable(true).withName("Reset Tunables"));

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
        visionFieldObjects = new FieldObject2d[VisionConstants.LIMELIGHT_NAMES.length];
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            visionHasTargetKeys[i] = "Vision/" + VisionConstants.LIMELIGHT_NAMES[i] + " Has Target";
            visionFieldObjects[i] = field.getObject("Vision " + VisionConstants.LIMELIGHT_NAMES[i]);
        }

        // --- Limelight camera streams ---
        // Registers each Limelight's MJPEG stream under /CameraPublisher so
        // Elastic's Camera Stream widget can display it. The dashboard pulls
        // video straight from the camera; nothing streams through the roboRIO.
        // (Nothing is registered while LIMELIGHT_NAMES is empty.)
        for (String name : VisionConstants.LIMELIGHT_NAMES) {
            CameraServer.addCamera(new HttpCamera(
                "limelight-" + name,
                "http://limelight-" + name + ".local:5800/stream.mjpg",
                HttpCameraKind.kMJPGStreamer));
        }
    }

    /**
     * Publishes all live values. Called every loop from Robot.robotPeriodic().
     */
    public void update() {
        // --- Field + drivetrain ---
        // A private snapshot: getState() returns the object the odometry
        // thread rewrites, so pose/speeds/modules read below would otherwise
        // come from different odometry ticks
        var driveState = swerve.getStateCopy();
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
        // (Vision/BestTag and Vision/AlignmentTag are logged in the Vision
        // section below, beside the dashboard values they mirror.)

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
        SmartDashboard.putBoolean("AlLow/Holding", allow.isHoldingPosition());
        SmartDashboard.putNumber("AlLow/Pivot Current", allow.getPivotCurrent());
        SmartDashboard.putNumber("AlLow/Roller Current", allow.getRollerCurrent());
        SmartDashboard.putNumber("AlLow/Pivot Output", allow.getPivotOutput());
        SmartDashboard.putNumber("AlLow/Roller Output", allow.getRollerOutput());

        // --- KitBot ---
        SmartDashboard.putNumber("KitBot/Roller Current", kitbot.getRollerCurrent());
        SmartDashboard.putNumber("KitBot/Roller Output", kitbot.getRollerOutput());

        // --- Vision (every value reads "nothing seen" until a Limelight is configured) ---
        // What the cameras SEE, unfiltered - the same tags their streams
        // draw: the closest one ("Best Tag"), which camera has it, and every
        // ID per camera. These deliberately do NOT come from the alignment
        // cache below: that cache skips a camera whose lens pose is
        // unmeasured, every tag outside a camera's alignment class and any
        // frame without a 3D solve, so a readout built on it can sit still
        // while a stream plainly shows a tag.
        var vision = swerve.getVision();
        var seenTag = vision.getClosestSeenTag();
        SmartDashboard.putNumber("Vision/Best Tag", seenTag.map(t -> (double) t.id).orElse(-1.0));
        SmartDashboard.putString("Vision/Best Tag Camera",
            seenTag.map(t -> VisionConstants.LIMELIGHT_NAMES[t.cameraIndex]).orElse(""));
        SmartDashboard.putString("Vision/Visible Tags", vision.getSeenTagsSummary());
        // The closest tag a camera may ALIGN on (its class, lens pose
        // measured), ignoring the alignment class and the latch so the
        // readouts work with the robot pushed into position while disabled;
        // positions are in the ROBOT frame. TX, Distance, Lateral and Square
        // Heading below describe THIS tag, not Best Tag. -1 while Best Tag
        // shows an ID = that tag is seen but is not one its camera may align on.
        var bestTarget = vision.getBestVisibleTarget();
        SmartDashboard.putNumber("Vision/Alignment Tag",
            bestTarget.map(t -> (double) t.id).orElse(-1.0));
        SmartDashboard.putNumber("Vision/TX",
            bestTarget.map(t -> t.tx).orElse(0.0));
        // Forward distance from the robot center to the tag (negative = behind)
        SmartDashboard.putNumber("Vision/Distance",
            bestTarget.map(t -> t.robotFrame.getX()).orElse(0.0));
        // Lateral position of the tag (+ = to the robot's left)
        SmartDashboard.putNumber("Vision/Lateral",
            bestTarget.map(t -> t.robotFrame.getY()).orElse(0.0));
        SmartDashboard.putNumber("Vision/Square Heading",
            bestTarget.flatMap(t -> t.squareHeading).map(Rotation2d::getDegrees).orElse(0.0));
        Logger.recordOutput("Vision/BestTag", seenTag.map(t -> t.id).orElse(-1));
        Logger.recordOutput("Vision/AlignmentTag", bestTarget.map(t -> t.id).orElse(-1));
        for (int i = 0; i < visionHasTargetKeys.length; i++) {
            SmartDashboard.putBoolean(visionHasTargetKeys[i], vision.hasTarget(i));
            // Last pose fused from each camera, drawn on the field beside the
            // robot; cleared once that camera has not fused for a second
            final int camera = i;
            vision.getLastFusedPose(i).ifPresentOrElse(
                visionFieldObjects[camera]::setPose,
                () -> visionFieldObjects[camera].setPoses(java.util.List.of()));
        }
        // What the driver is aligning on (NONE unless an align trigger is held)
        SmartDashboard.putString("Vision/Alignment Class", vision.getAlignmentClass().name());
        SmartDashboard.putNumber("Vision/Latched Tag", vision.getLatchedTagId());
        SmartDashboard.putNumber("Vision/Heading Offset", vision.getLatchedHeadingOffsetDegrees());
        SmartDashboard.putBoolean("Vision/Heading Seed Fresh", vision.hasFreshHeadingSeed());
        SmartDashboard.putBoolean("Vision/Reseeding Heading", vision.isReseedingHeading());
        SmartDashboard.putBoolean("Vision/Auto Kept Heading", swerve.lastAutoResetKeptHeading());
        // Alignment servo state (robot frame)
        SmartDashboard.putBoolean("Vision/Target Visible", swerve.isAlignmentTargetVisible());
        SmartDashboard.putBoolean("Vision/Target From Memory",
            vision.getBestTarget().map(t -> t.fromMemory).orElse(false));
        SmartDashboard.putBoolean("Vision/Aligned", swerve.isAligned());
        SmartDashboard.putNumber("Vision/Forward Error", swerve.getAlignmentForwardError());
        SmartDashboard.putNumber("Vision/Lateral Error", swerve.getAlignmentLateralError());
        SmartDashboard.putNumber("Vision/Heading Error", swerve.getAlignmentHeadingErrorDegrees());

        // --- Alerts (persistent conditions) ---
        // Resting-voltage check only while disabled - voltage sags under
        // load during a match are normal and would nag the drive team.
        lowBatteryAlert.set(DriverStation.isDisabled()
            && RobotController.getBatteryVoltage() < 12.0);
    }
}
