package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Robot-wide numerical and boolean constants, grouped by subsystem.
 * This class should not be used for any other purpose - all constants are
 * declared globally (public static) and nothing in here is functional code.
 *
 * Conventions used throughout this file:
 *  - Every value carries its unit in the comment (degrees, amps, m/s, ...)
 *  - Spark MAX PID gains are duty-cycle-based with voltage feedforward,
 *    as noted per section
 *  - Values marked TUNE are safe starting points for on-robot testing;
 *    values marked VERIFY or MEASURE are physical measurements that must
 *    be confirmed before the gains on top of them mean anything
 *  - Values that get dialed in on the robot are also dashboard tunables
 *    (util/Tunables.java); the number here is then only the default
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
 *
 * Swerve drivetrain constants live separately in generated/TunerConstants.java
 * because that file follows CTRE Tuner X's generated format.
 */
public final class Constants {

	/**
	 * Constants for the KitBot roller subsystem.
	 * Hardware: 1x NEO on a Spark MAX driving the coral eject roller.
	 * Negative duty cycle ejects a piece; positive pulls it back in.
	 */
	public static final class KitBotConstants {
		// --- CAN IDs ---
		public static final int ROLLER_MOTOR_ID = 5; // Roller Spark MAX

		// --- Current Limit (amps) ---
		public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60; // Spark MAX smart current limit

		// --- Voltage Compensation ---
		public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10; // Volts; keeps eject speed consistent as the battery sags

		// --- Roller Speeds (duty cycle, -1 to 1; negative = eject) ---
		public static final double ROLLER_FIRST_EJECT_VALUE = -0.2;   // Ejecting the first (single) piece
		public static final double ROLLER_STACKED_EJECT_VALUE = -0.3; // Ejecting a stacked piece (needs more push)
		public static final double ROLLER_REALIGN_VALUE = 0.1;        // Pulling a piece back in to re-seat it
		public static final double ROLLER_JOG_VALUE = -0.02;          // Slow creep for fine positioning

		// --- Timing (seconds) ---
		public static final double ROLLER_EJECT_DURATION = 0.5; // How long the eject commands run the roller
	}

	/**
	 * Constants for the AlLow (Algae Low) ground intake subsystem.
	 * Hardware: NEO on Spark MAX for the pivot, NEO on Spark MAX for the
	 * rollers. The pivot is zeroed at its stowed position on initialization,
	 * so stowed IS zero degrees and deploy angles increase from there.
	 */
	public static final class AlLowConstants {
		// --- CAN IDs ---
		public static final int ALLOW_PIVOT_MOTOR_ID = 61;  // Pivot Spark MAX
		public static final int ALLOW_ROLLER_MOTOR_ID = 62; // Roller Spark MAX

		// --- Motor Inversion ---
		public static final boolean ALLOW_PIVOT_MOTOR_INVERTED = false;  // True if positive output should be flipped (positive must deploy the arm)
		public static final boolean ALLOW_ROLLER_MOTOR_INVERTED = false; // True if positive output should be flipped (positive must eject)

		// --- Current Limits (amps) ---
		public static final int ALLOW_PIVOT_CURRENT_LIMIT = 30;  // Spark MAX smart current limit
		public static final int ALLOW_ROLLER_CURRENT_LIMIT = 20; // Spark MAX smart current limit

		// --- Encoder Conversion Factors ---
		// Degrees of arm travel per MOTOR rotation. 13.334 deg/rot implies a
		// ~27:1 reduction (360 / 13.334). VERIFY against the real gear train:
		// command 45 degrees and check the arm with a protractor - if it is
		// off, none of the gains below mean anything.
		public static final double ALLOW_PIVOT_POSITION_CONVERSION = 13.334;        // Degrees per motor rotation
		public static final double ALLOW_PIVOT_VELOCITY_CONVERSION = 13.334 / 60.0; // Degrees per second per motor RPM

		// --- Closed-Loop PID Gains (Spark MAX slot 0; error in degrees) ---
		// TUNE - first power-on procedure:
		//   1. Sanity-check the angle reading (see conversion note above).
		//   2. Raise kG until the arm just holds at the INTAKE angle at rest.
		//   3. Command a preset and raise kP until tracking is crisp;
		//      add kD only if it oscillates.
		public static final double ALLOW_PIVOT_kP = 0.01; // Duty cycle per degree of position error
		public static final double ALLOW_PIVOT_kI = 0.0;  // Leave 0 - kG handles gravity (integral just winds up)
		public static final double ALLOW_PIVOT_kD = 0.0;  // Duty cycle per deg/s of error derivative

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

		// --- Voltage Compensation ---
		public static final double ALLOW_NOMINAL_VOLTAGE = 12.0; // Volts; keeps response consistent as the battery sags

		// --- Pivot Angle Limits (degrees; enforced as controller soft limits) ---
		public static final double ALLOW_PIVOT_MIN_ANGLE = 0.0;  // Reverse soft limit (stowed)
		public static final double ALLOW_PIVOT_MAX_ANGLE = 60.0; // Forward soft limit (fully deployed)

		// --- Tolerances (degrees) ---
		public static final double ALLOW_PIVOT_ALLOWED_ERROR = 5.0; // "At target" threshold

		// --- Manual Control (unitless stick values) ---
		public static final double ALLOW_MANUAL_CONTROL_DEADBAND = 0.2; // Stick deadband
		public static final double ALLOW_MANUAL_SPEED_LIMIT = 0.15;     // Max duty cycle in manual mode
		// When the stick is released the arm is still moving, so the hold
		// target is the measured angle plus the distance the arm covers
		// while it stops: measured velocity x this time / 2 (a constant
		// deceleration to rest over this time). Holding the measured angle
		// itself would let the arm coast past its target and be pulled back.
		// TUNE: release the stick mid-travel and watch AlLow/Angle against
		// AlLow/Target - raise this if the arm comes BACK after a release,
		// lower it if it keeps creeping on.
		public static final double ALLOW_MANUAL_RELEASE_STOP_SECONDS = 0.15; // Seconds - TUNE

		// --- Encoder Zeroing ---
		// How long the operator's Start button must be held (robot DISABLED,
		// arm at its stow) before the pivot encoder is zeroed, so a brushed
		// button cannot move the arm's reference frame.
		public static final double ALLOW_ZERO_HOLD_SECONDS = 1.0;

		// --- Roller Speeds (duty cycle, -1 to 1; negative = intake) ---
		public static final double ALLOW_ROLLER_INTAKE_SPEED = -0.3; // Pulling algae in
		public static final double ALLOW_ROLLER_EJECT_SPEED = 0.6;   // Pushing algae out

		// --- Preset Angles (degrees; 0 = stowed) ---
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
	}

	/**
	 * Constants for teleop driving.
	 */
	public static final class DriveConstants {
		// Fraction of the drivetrain's theoretical top speed the translation
		// stick commands at full deflection, and of MAX_ANGULAR_RATE the
		// rotation stick commands. This is the DEFAULT for the "Drive -
		// Teleop Speed Scale" tunable, which can be changed from the
		// dashboard without a redeploy: lower it (0.25) for indoor testing,
		// raise it toward 1.0 as the drivers are ready.
		public static final double TELEOP_SPEED_SCALE = 0.75;

		// Rotation rate at full stick and a speed scale of 1.0, in rotations
		// per second (0.75 rot/s at the default scale).
		public static final double MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND = 1.0;

		// Stick deadband as a fraction of the SCALED top speed (so it stays
		// 20% of stick travel at every speed scale).
		public static final double STICK_DEADBAND = 0.2;

		// Speed of the slow robot-centric D-pad nudges
		public static final double NUDGE_SPEED_METERS_PER_SECOND = 0.5;

		// How far a driver trigger must be pulled (0-1) before its
		// hold-to-align binding counts as held
		public static final double ALIGN_TRIGGER_THRESHOLD = 0.3;

		// Driver rumble strength (0-1) while an alignment is held and aligned
		public static final double ALIGNED_RUMBLE_STRENGTH = 0.5;
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
		// PathPlanner's recommended starting point is 5.0 if they need retuning.
		public static final double TRANSLATION_kP = 10.0;
		public static final double TRANSLATION_kI = 0.0; // Leave 0 - integral winds up over long paths
		public static final double TRANSLATION_kD = 0.0;

		// --- Rotation Controller (rad/s of correction per radian of error) ---
		public static final double ROTATION_kP = 7.0;
		public static final double ROTATION_kI = 0.0;
		public static final double ROTATION_kD = 0.0;
	}

	/**
	 * Constants for the Vision subsystem (Limelight) and AprilTag alignment.
	 *
	 * THIS ROBOT CURRENTLY HAS NO LIMELIGHTS MOUNTED, so the three per-camera
	 * arrays below are empty and the whole vision stack is inert: nothing is
	 * written to or read from NetworkTables and the driver's align bindings
	 * are not created. Enabling a camera is a constants-only change:
	 *   1. mount it (Limelight OS 2026.0+, AprilTag pipeline at index 0) and
	 *      add its short name to LIMELIGHT_NAMES;
	 *   2. list the tag classes it may ALIGN on in LIMELIGHT_TRACKING_CLASSES
	 *      (a rear camera for the reef, a front camera for the coral station
	 *      - see the approach sides below);
	 *   3. MEASURE the lens pose, enter it in LIMELIGHT_POSES and mark it
	 *      measured = true. Until then the camera supplies no alignment
	 *      target and no heading seed.
	 * The commented example beside each array shows the shape of an entry.
	 *
	 * Limelight camera space (what the camera reports for a tag):
	 *   X = right of the camera, Y = down, Z = forward out of the lens.
	 * Alignment works in the ROBOT frame (WPILib: +X forward, +Y left,
	 * origin at the robot center); Vision converts between the two with the
	 * camera's mounting pose.
	 */
	public static final class VisionConstants {
		// --- Limelight Names (NetworkTables: "limelight-" + name) ---
		// Empty until a camera is mounted on this robot.
		// Example: {"front", "rear"}
		public static final String[] LIMELIGHT_NAMES = {};

		// --- Pipelines ---
		public static final int APRILTAG_PIPELINE = 0; // Pipeline index used for AprilTag detection

		// --- Thermal Management ---
		// Limelight "throttle": the camera processes one frame, then skips
		// this many. While the robot is DISABLED (pit, queue, between
		// periods - most of the camera's powered-on life) full-rate AprilTag
		// processing is wasted work that just heats the camera and spins the
		// fan; Limelight's guidance is 100-200. Full rate (0) is restored
		// the instant the robot enables, so tracking performance is
		// unaffected. At ~90 fps, 100 still yields ~1 solve/s while disabled,
		// plenty for the pre-match heading seed. Selecting Test mode on the
		// Driver Station also lifts the throttle while disabled, so the
		// dashboard's vision readouts are live on the bench.
		public static final int DISABLED_THROTTLE = 100;
		public static final int ENABLED_THROTTLE = 0;
		// The dashboard's seen-tag readouts ("Vision/Best Tag", "Vision/Best
		// Tag Camera", "Vision/Visible Tags") ignore a camera whose tag list
		// has not changed for this long (NetworkTables keeps the last value
		// of a camera that lost power or its link). Longer than the gap
		// between solves at the disabled throttle.
		public static final double SEEN_TAG_STALE_SECONDS = 10.0;

		// --- Camera Roles ---
		// Which tag classes each Limelight may supply to the ALIGNMENT
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
		// KitBot roller ejects out of the robot's REAR (-X), so the reef is
		// backed up to; the coral is loaded at the FRONT (+X), so a coral
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
		// a camera sees into where the ROBOT is, and alignment uses it to
		// convert a tag from camera space into the robot frame, so an error
		// here shifts every fused pose and every alignment from that camera.
		// Pushed to the camera at startup (so it lives in version control and
		// survives a camera reset), but ONLY for cameras marked measured -
		// unmeasured cameras keep whatever their web UI holds.
		//
		// Limelight robot-space convention (docs "3D Coordinate Systems"):
		// origin at the frame center projected to the floor; X+ forward,
		// Y+ toward the robot's RIGHT (note: opposite of WPILib's +Y = left),
		// Z+ up; meters and degrees. Pitch is entered as positive = lens
		// tilted UP, yaw as the heading of the lens (180 = rear-facing).
		// MEASURE with a tape from the frame center to the front of the lens,
		// and the tilt with an inclinometer on the camera body. When entering
		// a new pose, VERIFY the pitch/yaw signs against the camera's web-UI
		// 3D preview (it mirrors these values) - if the preview shows the
		// camera pointing the wrong way, flip the sign here.
		public static final class CameraPose {
			public final double forwardMeters, sideMeters, upMeters;
			public final double rollDegrees, pitchDegrees, yawDegrees;
			/** False = placeholder: the pose is NOT pushed to the camera, and the camera supplies no alignment target or heading seed. */
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
			 * +Y left): Limelight's side axis is positive to the RIGHT.
			 */
			public Translation2d lensOnRobot() {
				return new Translation2d(forwardMeters, -sideMeters);
			}

			/**
			 * True when the lens looks out of the BACK half of the robot.
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
		// gate use, and the field size PathPlanner's alliance flip must use.
		// Welded is the standard event field; switch to
		// k2025ReefscapeAndyMark for an AndyMark field.
		public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded;
		public static final double FIELD_LENGTH_METERS = 17.548;
		public static final double FIELD_WIDTH_METERS = 8.052;
		// A fused pose may lie this far outside the field before it is rejected
		public static final double FIELD_BOUNDS_MARGIN_METERS = 0.5;

		// --- Pose-estimate plausibility gates ---
		// Single-tag MegaTag1 solves (the heading seed) are only trusted when
		// unambiguous and close; these are Limelight's documented thresholds.
		public static final double MT1_SINGLE_TAG_MAX_AMBIGUITY = 0.7;
		public static final double MT1_SINGLE_TAG_MAX_DISTANCE_METERS = 3.0;
		// Heading std devs (rad) for MegaTag1 while seeding. The estimator only
		// closes part of the heading error per fused solve (about 2/3 at 0.05
		// against its 0.1 rad state std dev), and the disabled throttle yields
		// ~1 solve/s, so 2+ tag solves are trusted tightly to converge in a
		// few seconds; a lone tag (already gated on ambiguity/distance) less so.
		public static final double MT1_MULTI_TAG_ROTATION_STD_DEV = 0.05;
		public static final double MT1_SINGLE_TAG_ROTATION_STD_DEV = 0.3;
		// MegaTag2 translations from very far tags add little and can jump
		public static final double MT2_MAX_AVG_TAG_DISTANCE_METERS = 6.0;
		// How recently a trusted MegaTag1 solve must have been fused for the
		// heading to count as field-referenced (auto start then keeps it
		// instead of the path's nominal heading), and how far it may disagree
		// with the nominal heading before the nominal one wins.
		public static final double HEADING_SEED_FRESHNESS_SECONDS = 3.0;
		public static final double HEADING_SEED_MAX_DISAGREEMENT_DEGREES = 20.0;
		// ...and the estimator heading must have CONVERGED on that solve: it
		// must agree with the solve's own heading within this much
		public static final double HEADING_SEED_AGREEMENT_DEGREES = 3.0;
		// Heading re-seed while enabled (driver Y, Vision.requestHeadingReseed):
		// MegaTag1 is fused for this long so the pose heading corrects from
		// tag geometry
		public static final double HEADING_RESEED_WINDOW_SECONDS = 2.0;

		// --- Tag Classes (2025 Reefscape field) ---
		// Which tag IDs belong to each alignment class. Barge (4, 5, 14, 15)
		// and processor (3, 16) tags are in neither list: this robot has no
		// mechanism for them, so they have class NONE and the alignment
		// tracker ignores them. (All tags still feed MegaTag and the
		// dashboard's seen-tag readouts.)
		public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
		public static final int[] CORAL_STATION_TAGS = {1, 2, 12, 13};

		// --- Tracking Goal Distances (meters, ROBOT frame) ---
		// How far the tag sits from the robot CENTER when the robot is in
		// position; the tracker works in the robot frame, so these are
		// geometry, not camera readings. Flush = the tag's face at the bumper
		// face: half the bumper-to-bumper length (0.927 m / 2 = 0.464 m, the
		// robot size in deploy/pathplanner/settings.json) plus a little
		// standoff. They are the DEFAULTS of the "Vision - ... Flush Distance"
		// tunables. TUNE by pushing the robot into position and copying the
		// magnitude of the Vision/Distance reading.
		public static final double REEF_FLUSH_DISTANCE = 0.47;    // REAR bumper flush with the reef base, centered on the face (KitBot L1 eject)
		public static final double STATION_FLUSH_DISTANCE = 0.47; // FRONT bumper flush with the coral station wall, centered

		// A camera-space Z (forward) below this is not a real tag solve (an
		// empty or zeroed targetpose array)
		public static final double MIN_CAMERA_Z = 0.1;

		// --- Tracking Gains and Limits ---
		// The servo and its starting numbers come from the A robot, which
		// shares this chassis and drivetrain; every value marked TUNE is to
		// be confirmed on THIS robot the first time a camera is mounted.
		public static final class TrackingGains {
			// Defaults of the "Vision - Tracking ... kP" tunables
			public static final double DISTANCE_kP = 1.5;  // TUNE - m/s of drive per meter of position error
			public static final double ROTATION_kP = 0.06; // TUNE - rad/s of rotation per DEGREE of heading error

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

			// Commands are slew-limited: an unlimited P output STEPS (0 to
			// 2 m/s on the first loop, and to zero the instant a frame is
			// missed), which is wheel slip and a lurch. The P law itself
			// never asks for more than kP x speed of deceleration
			// (1.5 x 2 = 3 m/s^2), so the limit does not cause overshoot on
			// the way in.
			public static final double MAX_LINEAR_ACCELERATION = 3.0;  // m/s^2
			public static final double MAX_ANGULAR_ACCELERATION = 6.0; // rad/s^2

			// Once inside the deadbands the robot holds still until an error
			// grows past deadband x this. Without the gap, an error sitting ON
			// a deadband edge toggles the command every loop.
			public static final double DEADBAND_EXIT_RATIO = 1.6;

			// Filter on the latched tag's FIELD position (Vision class note):
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

			// Flush means the bumper is ON the reef or the wall, and a
			// closed-loop velocity command into a wall is a stalled drivetrain
			// stuttering against it. If the robot is being told to move, is not
			// moving, is laterally in position and is within this much of the
			// forward goal for this long, it has arrived by contact.
			public static final double CONTACT_FORWARD_ERROR = 0.10; // Meters
			public static final double CONTACT_MAX_SPEED = 0.04;     // m/s measured
			public static final double CONTACT_SECONDS = 0.3;

			// The tracker keeps its first tag, and while that tag is out of
			// view carries its last sighting on odometry, for this long: the
			// goal cannot flip between adjacent reef faces, and an approach
			// whose tag leaves the camera's view in the last stretch (a tag
			// fills and then leaves the image as the bumper closes on it)
			// still finishes on dead reckoning.
			public static final double TARGET_MEMORY_SECONDS = 1.5;
		}
	}
}
