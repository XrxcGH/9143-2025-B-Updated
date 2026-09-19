package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Every changeable number and setting on the robot, grouped by subsystem.
 * This is the one file a student edits to tune the robot: every other class
 * is logic only and reads its values from here. Nothing in here is
 * functional code (the two small helpers on CameraPose and the preset enum
 * only describe their own data).
 *
 * What deliberately lives elsewhere:
 *  - Swerve drivetrain constants: generated/TunerConstants.java, because
 *    that file follows CTRE Tuner X's generated format
 *  - Names and ports that other software expects, in the class that uses
 *    them: NetworkTables topic names and Preferences key strings (renaming
 *    a Preferences key loses its stored value, and the Elastic layout
 *    binds to the topic names), the Elastic tab names and the layout
 *    server's port 5800 (Robot), the PathPlanner named-command names the
 *    .auto files reference (RobotContainer), and the Limelight's table-name
 *    prefix, stream port and NetworkTables array layouts (Vision,
 *    Dashboard)
 *  - True mathematical and unit constants and fixed conventions (360 deg
 *    per turn, 1e6 microseconds per second, the 20 ms robot loop, the
 *    alliance perspectives, Telemetry's unit-sized module drawings)
 *  - Vendor files (LimelightHelpers.java, util/Elastic.java)
 *
 * ============================== CONTENTS ================================
 *   Nested class           Configures                         Read by
 *   KitBotConstants        KitBot roller: CAN id and timeout, KitBot
 *                          inversion, current limit, speeds,
 *                          eject time
 *   AlLowConstants         AlLow pivot and rollers: CAN ids,  AlLow,
 *                          gains, limits, speeds, presets,    RobotContainer,
 *                          zeroing hold, arm length,          Dashboard
 *                          simulation model
 *   DriveConstants         Teleop driving: speed scale and    RobotContainer,
 *                          its limits, rotation rate,         Tunables
 *                          deadband, nudges, align trigger,
 *                          aligned rumble
 *   ControllerConstants    Driver and operator controller     RobotContainer
 *                          ports, operator trigger threshold
 *   SwerveConstants        Swerve subsystem settings outside  Swerve
 *                          TunerConstants: SysId steps,
 *                          simulation loop period
 *   AutoConstants          Path-following gains, default auto Swerve,
 *                                                             RobotContainer
 *   VisionConstants        Limelights, field, pose-fusion     Vision, Swerve,
 *                          gates and confidence, alignment    Dashboard,
 *                          goals and their limits             Tunables,
 *                                                             RobotContainer
 *     .TrackingGains       Alignment servo gains, limits,     Swerve, Vision,
 *                          slew, filters, contact detection   Tunables
 *   TunablesConstants      Version stamp of the dashboard-    Tunables
 *                          tunable defaults
 *   DashboardConstants     Log server port and metadata, low  Robot, Dashboard
 *                          battery alert, AlLow arm drawing
 * ========================================================================
 *
 * Conventions used throughout this file:
 *  - Every value that has a unit gives it in its name or its comment
 *    (degrees, amps, m/s, ...)
 *  - Spark MAX PID gains are duty-cycle-based with voltage feedforward,
 *    as noted per section
 *  - Values marked TUNE are safe starting points for on-robot testing;
 *    values marked VERIFY or MEASURE are physical measurements that must
 *    be confirmed before the gains on top of them mean anything. A value
 *    with no marker (a filter constant, a plausibility gate, a drawing
 *    size) is a design choice rather than a property of this robot:
 *    change it only to change the behavior it sets
 *  - Values that get dialed in on the robot are also dashboard tunables
 *    (util/Tunables.java); the number here is then only the default, and
 *    TunablesConstants.DEFAULTS_VERSION decides whether a changed default
 *    reaches a robot that already has a value stored
 *
 * ============================== CAN ID MAP ==============================
 *   2       Pigeon 2 IMU                     (rio bus)
 *   5       KitBot roller Spark MAX
 *   11/12/13  Back Left  swerve drive / steer / CANcoder
 *   21/22/23  Front Left swerve drive / steer / CANcoder
 *   31/32/33  Front Right swerve drive / steer / CANcoder
 *   41/42/43  Back Right swerve drive / steer / CANcoder
 *   61/62   AlLow pivot / roller Spark MAX
 * ========================================================================
 */
public final class Constants {

    /**
     * Constants for the KitBot roller subsystem.
     * Hardware: one NEO on a Spark MAX driving the coral eject roller.
     * Negative duty cycle ejects a piece; positive pulls it back in.
     */
    public static final class KitBotConstants {
        // --- CAN IDs (VERIFY in the REV Hardware Client) ---
        public static final int ROLLER_MOTOR_ID = 5; // Roller Spark MAX

        // --- CAN Configuration ---
        // How long each configuration call waits for the Spark MAX to
        // answer (milliseconds). Parameters are only set once on
        // construction, so the timeout can be long without blocking robot
        // operation. VERIFY no CAN timeout errors at boot.
        public static final int ROLLER_CAN_TIMEOUT_MS = 250;

        // --- Motor Inversion (VERIFY on the robot) ---
        public static final boolean ROLLER_MOTOR_INVERTED = false; // True if positive output should be flipped (negative must eject)

        // --- Idle Mode ---
        // Not set by the code: the configuration reset at boot leaves a
        // Spark MAX's idle mode alone, so the roller brakes or coasts as
        // stored on the controller. VERIFY it in the REV Hardware Client.

        // --- Current Limit (amps; VERIFY against the motor and breaker) ---
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60; // Spark MAX smart current limit

        // --- Voltage Compensation ---
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10; // Volts; keeps eject speed consistent as the battery sags

        // --- Roller Speeds (duty cycle, -1 to 1; negative = eject; TUNE) ---
        public static final double ROLLER_FIRST_EJECT_VALUE = -0.2;   // Ejecting the first (single) piece
        public static final double ROLLER_STACKED_EJECT_VALUE = -0.3; // Ejecting a stacked piece (needs more push)
        public static final double ROLLER_REALIGN_VALUE = 0.1;        // Pulling a piece back in to re-seat it
        public static final double ROLLER_JOG_VALUE = -0.02;          // Slow creep for fine positioning

        // --- Timing (seconds; TUNE) ---
        public static final double ROLLER_EJECT_DURATION = 0.5; // How long the eject commands run the roller
    }

    /**
     * Constants for the AlLow (Algae Low) ground intake subsystem.
     * Hardware: NEO on Spark MAX for the pivot, NEO on Spark MAX for the
     * rollers. The pivot is zeroed at its stowed position on initialization,
     * so stowed is zero degrees and deploy angles increase from there.
     */
    public static final class AlLowConstants {
        // --- CAN IDs (VERIFY in the REV Hardware Client) ---
        public static final int ALLOW_PIVOT_MOTOR_ID = 61;  // Pivot Spark MAX
        public static final int ALLOW_ROLLER_MOTOR_ID = 62; // Roller Spark MAX

        // --- Motor Inversion (VERIFY on the robot) ---
        public static final boolean ALLOW_PIVOT_MOTOR_INVERTED = false;  // True if positive output should be flipped (positive must deploy the arm)
        public static final boolean ALLOW_ROLLER_MOTOR_INVERTED = false; // True if positive output should be flipped (positive must eject)

        // --- Idle Modes ---
        // Brake on both motors: with no output the pivot resists being
        // back-driven by the arm's weight, and the rollers stop at once.
        public static final IdleMode ALLOW_PIVOT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode ALLOW_ROLLER_IDLE_MODE = IdleMode.kBrake;

        // --- Current Limits (amps; VERIFY against the motors and breakers) ---
        public static final int ALLOW_PIVOT_CURRENT_LIMIT = 30;  // Spark MAX smart current limit
        public static final int ALLOW_ROLLER_CURRENT_LIMIT = 20; // Spark MAX smart current limit

        // --- Encoder Conversion Factors ---
        // Degrees of arm travel per motor rotation. 13.334 deg/rot implies a
        // reduction of about 27:1 (360 / 13.334). VERIFY against the real gear train:
        // command 45 degrees and check the arm with a protractor - if it is
        // off, none of the gains below mean anything. The velocity factor is
        // derived from the position factor (change that, not this).
        public static final double ALLOW_PIVOT_POSITION_CONVERSION = 13.334;                                // Degrees per motor rotation
        public static final double ALLOW_PIVOT_VELOCITY_CONVERSION = ALLOW_PIVOT_POSITION_CONVERSION / 60.0; // Degrees per second per motor RPM

        // --- Closed-Loop PID Gains (Spark MAX slot 0; error in degrees) ---
        // TUNE - first power-on procedure:
        //   1. Sanity-check the angle reading (see conversion note above).
        //   2. Raise kG until the arm barely holds at the INTAKE angle at rest.
        //   3. Command a preset and raise kP until tracking is crisp;
        //      add kD only if it oscillates.
        public static final double ALLOW_PIVOT_kP = 0.01; // Duty cycle per degree of position error
        public static final double ALLOW_PIVOT_kI = 0.0;  // Leave 0 - kG handles gravity (integral only winds up)
        public static final double ALLOW_PIVOT_kD = 0.0;  // Duty cycle per deg/s of error derivative
        // Closed-loop output limits (duty cycle). Full range; TUNE: narrow
        // them to cap the pivot's power while the gains are first tuned.
        public static final double ALLOW_PIVOT_MIN_OUTPUT = -1;
        public static final double ALLOW_PIVOT_MAX_OUTPUT = 1;

        // --- Gravity Feedforward (volts) ---
        // Applied as arbitrary feedforward, scaled by sin(angle) so it is zero
        // with the arm stowed vertically (0 deg) and maximal with the arm
        // horizontal. VERIFY the stowed arm is actually vertical; TUNE by
        // raising until the arm holds the INTAKE angle with kP at 0.
        public static final double ALLOW_PIVOT_kG = 0.0;
        // There is deliberately no static-friction (kS) term. The pivot runs
        // plain position control, not MAXMotion, and no REV feedforward is
        // configured. If it is ever moved to MAXMotion with REV's feedforward:
        // REV applies +kS whenever the profile velocity is zero, which on a
        // gravity-loaded axis is a constant push in one direction at rest -
        // keep kS at 0 there and take kG from the mean of the volts needed to
        // creep up and to creep down.

        // --- Voltage Compensation (pivot only) ---
        public static final double ALLOW_NOMINAL_VOLTAGE = 12.0; // Volts; keeps response consistent as the battery sags

        // --- Pivot Angle Limits (degrees; enforced as controller soft limits; VERIFY the arm's real travel) ---
        public static final double ALLOW_PIVOT_MIN_ANGLE = 0.0;  // Reverse soft limit (stowed)
        public static final double ALLOW_PIVOT_MAX_ANGLE = 60.0; // Forward soft limit (fully deployed)

        // --- Tolerances (degrees; TUNE) ---
        public static final double ALLOW_PIVOT_ALLOWED_ERROR = 5.0; // "At target" threshold

        // --- Manual Control (unitless stick values; TUNE) ---
        public static final double ALLOW_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
        public static final double ALLOW_MANUAL_SPEED_LIMIT = 0.15;     // Max duty cycle in manual mode
        // When the stick is released the arm is still moving, so the hold
        // target is the measured angle plus the distance the arm covers
        // while it stops: measured velocity x this time / 2 (a constant
        // deceleration to rest over this time). Holding the measured angle
        // itself would let the arm coast past its target and be pulled back.
        // TUNE: release the stick mid-travel and watch AlLow/Angle against
        // AlLow/Target - raise this if the arm comes back after a release,
        // lower it if it keeps creeping on.
        public static final double ALLOW_MANUAL_RELEASE_STOP_SECONDS = 0.15; // Seconds - TUNE

        // --- Encoder Zeroing ---
        // How long the operator's Start button must be held (robot disabled,
        // arm at its stow) before the pivot encoder is zeroed, so a brushed
        // button cannot move the arm's reference frame. Seconds.
        public static final double ALLOW_ZERO_HOLD_SECONDS = 1.0;

        // --- Roller Speeds (duty cycle, -1 to 1; negative = intake; TUNE) ---
        // ALLOW_NOMINAL_VOLTAGE compensates the pivot only: the rollers run
        // plain duty cycle, so their speed falls as the battery sags.
        public static final double ALLOW_ROLLER_INTAKE_SPEED = -0.3; // Pulling algae in
        public static final double ALLOW_ROLLER_EJECT_SPEED = 0.6;   // Pushing algae out

        // --- Preset Angles (degrees; 0 = stowed; TUNE) ---
        public enum PivotPresetAngles {
            BASE(0.0),    // Stowed against the frame
            HOLD(15.0),   // Holding a piece clear of the ground
            INTAKE(45.0); // Deployed to ground-intake height

            private final double angle;

            PivotPresetAngles(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }

        // --- Arm Geometry ---
        // Pivot axis to the end of the arm (meters). The desktop simulation's
        // physics model and the dashboard's arm drawing both use it.
        public static final double ALLOW_ARM_LENGTH_METERS = 0.35; // Estimate - MEASURE

        // --- Desktop Simulation (affects the simulation only) ---
        // The physics model exists purely so the arm moves in the sim GUI and
        // AdvantageScope. It models no gravity (the note in AlLow's
        // constructor says why). Motor rotations per arm rotation, derived
        // from the conversion factor above (change that, not this).
        public static final double ALLOW_SIM_GEAR_RATIO = 360.0 / ALLOW_PIVOT_POSITION_CONVERSION;
        public static final double ALLOW_SIM_ARM_MASS_KG = 2.0; // Estimate - MEASURE
    }

    /**
     * Constants for teleop driving.
     */
    public static final class DriveConstants {
        // Fraction of the drivetrain's theoretical top speed the translation
        // stick commands at full deflection, and of MAX_ANGULAR_RATE the
        // rotation stick commands. This is the default for the "Drive -
        // Teleop Speed Scale" tunable, which can be changed from the
        // dashboard without a redeploy: lower it (0.25) for indoor testing,
        // raise it toward 1.0 as the drivers are ready.
        public static final double TELEOP_SPEED_SCALE = 0.75;
        // The range the tunable is clamped to, so a bad dashboard entry can
        // neither disable driving nor exceed the drivetrain's capability.
        public static final double TELEOP_SPEED_SCALE_MIN = 0.05;
        public static final double TELEOP_SPEED_SCALE_MAX = 1.0;

        // Rotation rate at full stick and a speed scale of 1.0, in rotations
        // per second (0.75 rot/s at the default scale). TUNE with the drivers.
        public static final double MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND = 1.0;

        // Stick deadband as a fraction of the scaled top speed (so it stays
        // 20% of stick travel at every speed scale). TUNE with the drivers.
        public static final double STICK_DEADBAND = 0.2;

        // Speed of the slow robot-centric D-pad nudges (m/s). TUNE.
        public static final double NUDGE_SPEED_METERS_PER_SECOND = 0.5;

        // How far a driver trigger must be pulled (0-1) before its
        // hold-to-align binding counts as held. TUNE.
        public static final double ALIGN_TRIGGER_THRESHOLD = 0.3;

        // Driver rumble strength (0-1) while an alignment is held and aligned. TUNE.
        public static final double ALIGNED_RUMBLE_STRENGTH = 0.5;
    }

    /**
     * Driver Station USB ports of the two Xbox controllers, and the
     * operator's trigger threshold. VERIFY the ports on the Driver Station's
     * USB tab: the driver's controller must be in slot 0.
     */
    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;   // Drivetrain and alignment
        public static final int OPERATOR_CONTROLLER_PORT = 1; // KitBot roller and AlLow

        // How far an operator trigger must be pulled (0-1) before its
        // hold-to-run AlLow roller binding counts as held. TUNE with the
        // operator. (The driver's align triggers use
        // DriveConstants.ALIGN_TRIGGER_THRESHOLD.)
        public static final double OPERATOR_TRIGGER_THRESHOLD = 0.5;
    }

    /**
     * Swerve subsystem settings that are not part of the drivetrain model.
     * The drivetrain itself (module gearing, CANcoder offsets, gains) is in
     * generated/TunerConstants.java.
     */
    public static final class SwerveConstants {
        // Period of the desktop-simulation thread (seconds). Faster than the
        // 20 ms robot loop so the simulated module PID gains behave more like
        // the real ones.
        public static final double SIM_LOOP_PERIOD_SECONDS = 0.005; // 5 ms

        // --- SysId characterization (Test mode only) ---
        // Dynamic step voltages. Translation is reduced to 4 V to prevent a
        // brownout. TUNE only if a routine hits a limit or browns out.
        public static final double SYSID_TRANSLATION_STEP_VOLTS = 4;
        public static final double SYSID_STEER_STEP_VOLTS = 7;
        // The rotation routine drives a rotation rate, not a voltage, but
        // SysId only records "volts": the ramp rate is in rad/s per second
        // and the step in rad/s.
        public static final double SYSID_ROTATION_RAMP_RATE = Math.PI / 6; // rad/s per second
        public static final double SYSID_ROTATION_STEP = Math.PI;          // rad/s
    }

    /**
     * Constants for autonomous path following (PathPlanner).
     * These gains are the feedback half of path following: PathPlanner already
     * feeds forward the velocities/accelerations from the path, and these PID
     * controllers correct whatever error remains between the robot's estimated
     * pose and the path. If they are too low the robot never pulls itself back
     * onto the path and finishes several feet off; too high and it oscillates.
     */
    public static final class AutoConstants {
        // --- Translation Controller (m/s of correction per meter of error) ---
        // These are the gains this robot's existing autos were driven with;
        // PathPlanner's recommended starting point is 5.0 if they need
        // retuning. TUNE on both alliances.
        public static final double TRANSLATION_kP = 10.0;
        public static final double TRANSLATION_kI = 0.0; // Leave 0 - integral winds up over long paths
        public static final double TRANSLATION_kD = 0.0;

        // --- Rotation Controller (rad/s of correction per radian of error; TUNE) ---
        public static final double ROTATION_kP = 7.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;

        // --- Auto Chooser ---
        // The auto selected when the dashboard chooser is first shown. It
        // must name an auto that exists in deploy/pathplanner/autos.
        public static final String DEFAULT_AUTO_NAME = "Center Drop";
    }

    /**
     * Constants for the Vision subsystem (Limelight) and AprilTag alignment.
     *
     * This robot currently has no Limelights mounted, so the three per-camera
     * arrays below are empty and the whole vision stack is inert: nothing is
     * written to or read from NetworkTables and the driver's align bindings
     * are not created. Enabling a camera is a constants-only change:
     *   1. mount it (Limelight OS 2026.0+, AprilTag pipeline at index 0) and
     *      add its short name to LIMELIGHT_NAMES;
     *   2. list the tag classes it may align on in LIMELIGHT_TRACKING_CLASSES
     *      (a rear camera for the reef, a front camera for the coral station
     *      - see the approach sides below);
     *   3. MEASURE the lens pose, enter it in LIMELIGHT_POSES and mark it
     *      measured = true. Until then the camera supplies no alignment
     *      target and no heading seed.
     * The commented example beside each array shows the shape of an entry.
     *
     * Limelight camera space (what the camera reports for a tag):
     *   X = right of the camera, Y = down, Z = forward out of the lens.
     * Alignment works in the robot frame (WPILib: +X forward, +Y left,
     * origin at the robot center); Vision converts between the two with the
     * camera's mounting pose.
     */
    public static final class VisionConstants {
        // --- Limelight Names (NetworkTables: "limelight-" + name) ---
        // Empty until a camera is mounted on this robot.
        // Example: {"front", "rear"}
        public static final String[] LIMELIGHT_NAMES = {};

        // --- Pipelines ---
        public static final int APRILTAG_PIPELINE = 0; // Pipeline index used for AprilTag detection - VERIFY in each camera's web UI

        // --- Thermal Management ---
        // Limelight "throttle": the camera processes one frame, then skips
        // this many. While the robot is disabled (pit, queue, between
        // periods - most of the camera's powered-on life) full-rate AprilTag
        // processing is wasted work that only heats the camera and spins the
        // fan; Limelight's guidance is 100-200. Full rate (0) is restored
        // the instant the robot enables, so tracking performance is
        // unaffected. At about 90 fps, 100 still yields about 1 solve/s while disabled,
        // plenty for the pre-match heading seed. Selecting Test mode on the
        // Driver Station also lifts the throttle while disabled, so the
        // dashboard's vision readouts are live on the bench.
        public static final int DISABLED_THROTTLE = 100;
        public static final int ENABLED_THROTTLE = 0;
        // The dashboard's seen-tag readouts ("Vision/Best Tag", "Vision/Best
        // Tag Camera", "Vision/Visible Tags") ignore a camera whose tag list
        // has not changed for this long (NetworkTables keeps the last value
        // of a camera that lost power or its link). Longer than the gap
        // between solves at the disabled throttle. Seconds.
        public static final double SEEN_TAG_STALE_SECONDS = 10.0;
        // Each camera's last fused pose is drawn on the Field widget until
        // that camera has not fused for this long (seconds).
        public static final double FUSED_POSE_DISPLAY_SECONDS = 1.0;

        // --- Camera Roles ---
        // Which tag classes each Limelight may supply to the alignment
        // tracker, same order as LIMELIGHT_NAMES (every camera still feeds
        // MegaTag pose estimation with whatever tags it sees). A class only
        // works on a camera that looks out of the side that class is
        // approached from (the approach sides below): REEF on a rear camera,
        // CORAL_STATION on a front camera. A class listed on a camera facing
        // the other way is ignored, with a warning at startup - the tag would
        // leave its view as the robot turns to square up.
        // Example, for {"front", "rear"}: {{TagClass.CORAL_STATION}, {TagClass.REEF}}
        public enum TagClass { REEF, CORAL_STATION, NONE }
        public static final TagClass[][] LIMELIGHT_TRACKING_CLASSES = {};

        // --- Approach Sides (this robot's mechanism layout) ---
        // Which end of the robot is driven up to each field element. The
        // KitBot roller ejects out of the robot's rear (-X), so the reef is
        // backed up to; the coral is loaded at the front (+X), so a coral
        // station is driven up to nose-first. These are taken from the
        // authored autos, which end every reef path with the robot's heading
        // equal to the way the reef tag faces (rear bumper on the reef) and
        // every station path facing the station wall (VisionGeometryTest
        // pins the two against each other). VERIFY ON THE ROBOT that "front"
        // here is the drivetrain's front: after a driver heading zero, stick
        // forward must lead with the end the coral is loaded from.
        public static final boolean REEF_APPROACH_REAR = true;
        public static final boolean STATION_APPROACH_REAR = false;

        // --- Camera Poses in Robot Space ---
        // Where each lens sits on the robot. MegaTag uses this to convert what
        // a camera sees into where the robot is, and alignment uses it to
        // convert a tag from camera space into the robot frame, so an error
        // here shifts every fused pose and every alignment from that camera.
        // Pushed to the camera at startup (so it lives in version control and
        // survives a camera reset), but only for cameras marked measured -
        // unmeasured cameras keep whatever their web UI holds.
        //
        // Limelight robot-space convention (docs "3D Coordinate Systems"):
        // origin at the frame center projected to the floor; X+ forward,
        // Y+ toward the robot's right (the opposite of WPILib's +Y = left),
        // Z+ up; meters and degrees. Pitch is entered as positive = lens
        // tilted up, yaw as the heading of the lens (180 = rear-facing).
        // MEASURE with a tape from the frame center to the front of the lens,
        // and the tilt with an inclinometer on the camera body. When entering
        // a new pose, VERIFY the pitch/yaw signs against the camera's web-UI
        // 3D preview (it mirrors these values) - if the preview shows the
        // camera pointing the wrong way, flip the sign here.
        public static final class CameraPose {
            public final double forwardMeters, sideMeters, upMeters;
            public final double rollDegrees, pitchDegrees, yawDegrees;
            /** False = placeholder: the pose is not pushed to the camera, and the camera supplies no alignment target or heading seed. */
            public final boolean measured;

            public CameraPose(double forwardMeters, double sideMeters, double upMeters,
                    double rollDegrees, double pitchDegrees, double yawDegrees, boolean measured) {
                this.forwardMeters = forwardMeters;
                this.sideMeters = sideMeters;
                this.upMeters = upMeters;
                this.rollDegrees = rollDegrees;
                this.pitchDegrees = pitchDegrees;
                this.yawDegrees = yawDegrees;
                this.measured = measured;
            }

            /**
             * The lens position in the WPILib robot frame (+X forward,
             * +Y left): Limelight's side axis is positive to the right.
             */
            public Translation2d lensOnRobot() {
                return new Translation2d(forwardMeters, -sideMeters);
            }

            /**
             * True when the lens looks out of the back half of the robot.
             * Such a camera can supply the tag classes that are approached
             * rear-first (backed up to), and only those.
             */
            public boolean facesRear() {
                return Math.cos(Math.toRadians(yawDegrees)) < 0.0;
            }
        }

        // Same order as LIMELIGHT_NAMES.
        // Example (placeholder numbers - MEASURE the real ones, then set the
        // last argument to true):
        //   new CameraPose(0.30, 0.0, 0.80, 0.0, 30.0, 0.0, false),    // front (coral station)
        //   new CameraPose(-0.30, 0.0, 0.25, 0.0, 15.0, 180.0, false), // rear (reef)
        public static final CameraPose[] LIMELIGHT_POSES = {};

        // --- Field (2025 Reefscape) ---
        // The AprilTag layout the alignment heading and the pose plausibility
        // gate use, and the field size and symmetry PathPlanner's alliance
        // flip must use (PathPlannerLib 2026 defaults to the 2026 field).
        // Welded is the standard event field; switch to
        // k2025ReefscapeAndyMark for an AndyMark field. VERIFY the field
        // type at each event.
        public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
        // The 2025 field is rotationally symmetric: the red alliance's
        // half is the blue half turned 180 deg about the field center.
        public static final FlippingUtil.FieldSymmetry FIELD_SYMMETRY = FlippingUtil.FieldSymmetry.kRotational;
        // A fused pose may lie this far outside the field before it is rejected
        public static final double FIELD_BOUNDS_MARGIN_METERS = 0.5;

        // --- Pose-estimate plausibility gates ---
        // A MegaTag1 solve from at least this many tags is multi-tag: it is
        // fused with MT1_MULTI_TAG_ROTATION_STD_DEV and is a strong heading
        // seed (it may set the driver's frame while disabled, and it lets
        // auto start keep the vision heading). A solve from fewer tags is
        // treated as single-tag: it must pass the ambiguity and distance gate
        // below (read from its first tag) and gets the looser
        // MT1_SINGLE_TAG_ROTATION_STD_DEV. Keep it at 2 or more: a lone
        // tag's solve can flip by tens of degrees.
        public static final int MT1_MULTI_TAG_MIN_COUNT = 2; // Tags
        // Single-tag MegaTag1 solves (the heading seed) are only trusted when
        // unambiguous and close; these are Limelight's documented thresholds.
        public static final double MT1_SINGLE_TAG_MAX_AMBIGUITY = 0.7;
        public static final double MT1_SINGLE_TAG_MAX_DISTANCE_METERS = 3.0;
        // Heading std devs (rad) for MegaTag1 while seeding. The estimator only
        // closes part of the heading error per fused solve (about 2/3 at 0.05
        // against its 0.1 rad state std dev), and the disabled throttle yields
        // about 1 solve/s, so multi-tag solves are trusted tightly to converge
        // in a few seconds; a lone tag (already gated on ambiguity/distance)
        // less so.
        public static final double MT1_MULTI_TAG_ROTATION_STD_DEV = 0.05;
        public static final double MT1_SINGLE_TAG_ROTATION_STD_DEV = 0.3;
        // MegaTag2 translations from very far tags add little and can jump
        public static final double MT2_MAX_AVG_TAG_DISTANCE_METERS = 6.0;
        // Vision poses are rejected while the robot spins faster than this
        // (rad/s) and for this long after (seconds): motion blur and rolling
        // shutter corrupt the solve, and the image is captured 25-100 ms
        // before it arrives, so the hold covers the last smeared frames.
        // TUNE if good frames are dropped during ordinary turning.
        public static final double VISION_MAX_OMEGA_RAD_PER_SEC = 2.0;
        public static final double VISION_REJECT_AFTER_SPIN_SECONDS = 0.2;
        // How recently a trusted MegaTag1 solve must have been fused for the
        // heading to count as field-referenced (auto start then keeps it
        // instead of the path's nominal heading), and how far it may disagree
        // with the nominal heading before the nominal one wins.
        public static final double HEADING_SEED_FRESHNESS_SECONDS = 3.0;
        public static final double HEADING_SEED_MAX_DISAGREEMENT_DEGREES = 20.0;
        // The estimator heading must also have converged on that solve: it
        // must agree with the solve's own heading within this much
        public static final double HEADING_SEED_AGREEMENT_DEGREES = 3.0;
        // Heading re-seed while enabled (driver Y, Vision.requestHeadingReseed):
        // MegaTag1 is fused for this long so the pose heading corrects from
        // tag geometry
        public static final double HEADING_RESEED_WINDOW_SECONDS = 2.0;

        // --- Pose-estimate confidence (standard deviations fed to the estimator) ---
        // Translation std dev (meters) = base + gain x (average tag
        // distance)^2 / tag count: close tags and many tags are trusted
        // more. TUNE if fused poses are jumpy (raise) or slow to correct
        // odometry drift (lower).
        public static final double XY_STD_DEV_BASE = 0.3;                // Meters
        public static final double XY_STD_DEV_DISTANCE_SQUARED_GAIN = 0.4; // Meters per square meter of average tag distance
        // MegaTag2's heading is our own gyro echoed back, so it gets
        // effectively zero weight (rad).
        public static final double MT2_ROTATION_STD_DEV = 9999999;

        // --- Tag Classes (2025 Reefscape field) ---
        // Which tag IDs belong to each alignment class. Barge (4, 5, 14, 15)
        // and processor (3, 16) tags are in neither list: this robot has no
        // mechanism for them, so they have class NONE and the alignment
        // tracker ignores them. (All tags still feed MegaTag and the
        // dashboard's seen-tag readouts.)
        public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
        public static final int[] CORAL_STATION_TAGS = {1, 2, 12, 13};

        // --- Tracking Goal Distances (meters, robot frame) ---
        // How far the tag sits from the robot center when the robot is in
        // position; the tracker works in the robot frame, so these are
        // geometry, not camera readings. Flush = the tag's face at the bumper
        // face: half the bumper-to-bumper length (0.927 m / 2 = 0.464 m, the
        // robot size in deploy/pathplanner/settings.json) plus a little
        // standoff. They are the DEFAULTS of the "Vision - ... Flush Distance"
        // tunables. TUNE by pushing the robot into position and copying the
        // magnitude of the Vision/Distance reading.
        public static final double REEF_FLUSH_DISTANCE = 0.47;    // Rear bumper flush with the reef base, centered on the face (KitBot L1 eject)
        public static final double STATION_FLUSH_DISTANCE = 0.47; // Front bumper flush with the coral station wall, centered
        // The range both flush-distance tunables are clamped to (meters,
        // taken by magnitude), so a typo on the dashboard cannot send the
        // robot into the reef or across the field.
        public static final double FLUSH_DISTANCE_MIN = 0.3;
        public static final double FLUSH_DISTANCE_MAX = 2.0;
        // Where the tag sits across the robot when it is in position (meters,
        // + = tag to the robot's left): 0.0 = centered. The L1 trough runs
        // the width of a reef face, so there is no left / right branch.
        // VERIFY that the KitBot roller (reef) and the coral opening
        // (station) are centered on the robot; if one is not, enter the
        // offset of its center from the robot center.
        public static final double REEF_GOAL_LEFT_METERS = 0.0;
        public static final double STATION_GOAL_LEFT_METERS = 0.0;

        // A camera-space Z (forward) below this is not a real tag solve (an
        // empty or zeroed targetpose array). Meters.
        public static final double MIN_CAMERA_Z = 0.1;

        // --- Tracking Gains and Limits ---
        // The servo and its starting numbers come from the A robot, which
        // shares this chassis and drivetrain; every value marked TUNE is to
        // be confirmed on this robot the first time a camera is mounted.
        public static final class TrackingGains {
            // Defaults of the "Vision - Tracking ... kP" tunables
            public static final double DISTANCE_kP = 1.5;  // TUNE - m/s of drive per meter of position error
            public static final double ROTATION_kP = 0.06; // TUNE - rad/s of rotation per degree of heading error
            // The range both kP tunables are clamped to: this minimum up to
            // this multiple of the default above.
            public static final double TUNABLE_KP_MIN = 0.0;
            public static final double TUNABLE_KP_MAX_FACTOR = 3.0;

            // Deadbands: errors below these are treated as zero. The L1
            // trough runs the width of a reef face and the station opening
            // is wide, so lateral needs no more precision than fore/aft,
            // which the reef base or the station wall absorbs when flush.
            public static final double FORWARD_ERROR_DEADBAND = 0.03;  // Meters - TUNE
            public static final double LATERAL_ERROR_DEADBAND = 0.03;  // Meters - TUNE
            public static final double ROTATION_ERROR_DEADBAND = 1.0;  // Degrees of heading error - TUNE

            // Smallest drive command outside a deadband: a P command below
            // this would not overcome static friction and the robot would
            // stall just outside the deadband. TUNE: the slowest speed at
            // which the robot still creeps steadily on carpet.
            public static final double MIN_LINEAR_VELOCITY = 0.12; // m/s
            public static final double MAX_LINEAR_VELOCITY = 2.0;  // m/s command clamp while tracking
            public static final double MAX_ANGULAR_VELOCITY = 1.0; // rad/s command clamp while tracking

            // Commands are slew-limited: an unlimited P output steps (0 to
            // 2 m/s on the first loop, and to zero the instant a frame is
            // missed), which is wheel slip and a lurch. The P law itself
            // never asks for more than kP x speed of deceleration
            // (1.5 x 2 = 3 m/s^2), so the limit does not cause overshoot on
            // the way in.
            public static final double MAX_LINEAR_ACCELERATION = 3.0;  // m/s^2
            public static final double MAX_ANGULAR_ACCELERATION = 6.0; // rad/s^2
            // The slew limiter's time step is the measured time since its
            // last command, bounded to this range (seconds). The upper bound
            // keeps the first loop of an alignment (the last command was long
            // ago) or a late loop from allowing a jump in speed; the lower
            // bound keeps the ramp moving if it runs twice in one loop.
            public static final double SLEW_MIN_DT_SECONDS = 0.005;
            public static final double SLEW_MAX_DT_SECONDS = 0.05;

            // Once inside the deadbands the robot holds still until an error
            // grows past deadband x this. Without the gap, an error sitting on
            // a deadband edge toggles the command every loop.
            public static final double DEADBAND_EXIT_RATIO = 1.6;

            // Filter on the latched tag's field position (Vision class note):
            // fraction of each new camera frame blended in. A sample further
            // than the outlier distance from the estimate is ignored unless it
            // persists for that many frames.
            public static final double TARGET_FILTER_ALPHA = 0.3;
            public static final double TARGET_OUTLIER_METERS = 0.25;
            public static final int TARGET_OUTLIER_FRAMES = 3;
            // Same idea for the offset between the field-true heading (from
            // each frame's MegaTag1 solve) and the pose estimator's heading.
            // Slower, with a wider gate that must persist longer: a single-tag
            // solve's heading is noisier than its position and can flip.
            public static final double HEADING_FILTER_ALPHA = 0.2;
            public static final double HEADING_OUTLIER_DEGREES = 12.0;
            public static final int HEADING_OUTLIER_FRAMES = 5;

            // Flush means the bumper is on the reef or the wall, and a
            // closed-loop velocity command into a wall is a stalled drivetrain
            // stuttering against it. If the robot is being told to move, is not
            // moving, is laterally in position and is within this much of the
            // forward goal for this long, it has arrived by contact.
            public static final double CONTACT_FORWARD_ERROR = 0.10; // Meters - TUNE
            public static final double CONTACT_MAX_SPEED = 0.04;     // m/s measured - TUNE
            public static final double CONTACT_SECONDS = 0.3;        // TUNE
            // "Told to move" = the last speed sent is at least this fraction
            // of MIN_LINEAR_VELOCITY: a margin below the minimum, so a command
            // sitting at the minimum always counts.
            public static final double CONTACT_COMMAND_RATIO = 0.9;

            // The tracker keeps its first tag, and while that tag is out of
            // view carries its last sighting on odometry, for this long: the
            // goal cannot flip between adjacent reef faces, and an approach
            // whose tag leaves the camera's view in the last stretch (a tag
            // fills and then leaves the image as the bumper closes on it)
            // still finishes on dead reckoning.
            public static final double TARGET_MEMORY_SECONDS = 1.5;
        }
    }

    /**
     * Constants for the dashboard-tunable values (util/Tunables.java). The
     * tunable defaults sit with their subsystems, each beside its clamp
     * range: DriveConstants.TELEOP_SPEED_SCALE,
     * VisionConstants.REEF_FLUSH_DISTANCE and STATION_FLUSH_DISTANCE, and
     * VisionConstants.TrackingGains.DISTANCE_kP and ROTATION_kP.
     */
    public static final class TunablesConstants {
        // Version stamp of those defaults. Stored values survive deploys, so
        // changing a default does nothing on a robot that already has the
        // key stored - unless this number is bumped. On the first boot after
        // a bump, Tunables.init() does one of two things:
        //   - if it has a targeted migration block for the stored version, it
        //     overwrites only the keys named there and keeps everything else
        //     the team has tuned on the dashboard;
        //   - otherwise it overwrites every tunable with the new defaults.
        // Bump it when a default changes and must take effect on the robot
        // (and add a migration block when only a few defaults moved); leave
        // it alone to preserve values tuned on the dashboard.
        // Version 1 is the first set of defaults.
        public static final int DEFAULTS_VERSION = 1;
    }

    /**
     * Constants for logging, dashboard alerts and the dashboard drawings.
     * None of these change how the robot moves.
     */
    public static final class DashboardConstants {
        // --- Logging (AdvantageKit) ---
        // Port of the live log stream for AdvantageScope's "Connect to
        // Robot" (RLOG). Not 5800, which the Elastic layout server owns;
        // both are inside the field-legal 5800-5810 range. VERIFY it matches
        // the RLOG port set in AdvantageScope.
        public static final int RLOG_PORT = 5810;
        // Metadata shown in AdvantageScope's metadata tab for every log
        public static final String LOG_PROJECT_NAME = "9143-2025-B";
        public static final String LOG_ROBOT_NAME = "B (KitBot + AlLow)";

        // --- Alerts ---
        // Resting battery voltage (volts, checked only while disabled - sag
        // under load is normal) below which the swap-the-battery Alert
        // shows. TUNE to the team's battery-swap rule.
        public static final double LOW_BATTERY_VOLTS = 12.0;

        // --- AlLow arm drawing (Mechanism2d, rendered by Glass and AdvantageScope) ---
        // A side-on schematic in meters: the canvas, where the pivot sits on
        // it, and how the arm line is drawn.
        public static final double ALLOW_MECHANISM_WIDTH = 1.2;  // Canvas width, meters
        public static final double ALLOW_MECHANISM_HEIGHT = 1.2; // Canvas height, meters
        public static final double ALLOW_MECHANISM_ROOT_X = 0.6; // Pivot, meters from the canvas's left edge
        public static final double ALLOW_MECHANISM_ROOT_Y = 0.2; // Pivot, meters above the canvas's bottom edge
        public static final double ALLOW_ARM_LINE_WIDTH = 6;     // Pixels
        public static final Color ALLOW_ARM_COLOR = Color.kCyan;
        // The arm is drawn AlLowConstants.ALLOW_ARM_LENGTH_METERS long.

        // --- AlLow arm 3D component pose (AdvantageScope 3D field view) ---
        // Attach a glTF CAD model and map the RobotState/ComponentPoses
        // entry to the arm component in the 3D config. Robot-relative frame:
        // X forward, Y left, Z up, origin at the robot center on the floor.
        // Offsets are placeholders - VERIFY against the CAD model's
        // component origin.
        public static final double ALLOW_PIVOT_X_OFFSET = 0.0; // Meters forward of robot center - VERIFY
        public static final double ALLOW_PIVOT_Y_OFFSET = 0;   // Meters left of robot center - VERIFY
        public static final double ALLOW_PIVOT_HEIGHT = 0.25;  // Pivot height above the floor (meters) - VERIFY
    }
}
