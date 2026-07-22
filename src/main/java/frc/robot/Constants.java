package frc.robot;

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
 *    values marked VERIFY are physical measurements that must be confirmed
 *    before the gains on top of them mean anything
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
	 * Constants for the Vision subsystem (Limelight) and AprilTag tracking.
	 *
	 * THIS ROBOT CURRENTLY HAS NO LIMELIGHTS MOUNTED. The vision stack is
	 * ported from the A robot and fully wired into the drivetrain; to enable
	 * it, mount a camera (LLOS 2026.0+), add its short name and facing sign
	 * to the two arrays below, and everything - MegaTag2 pose fusion, the
	 * tracking command, and the driver toggle - comes alive.
	 *
	 * Tracking coordinate frame (Limelight camera space):
	 *   X = right of the camera (+ means the tag appears to the robot's right)
	 *   Y = down (unused for driving)
	 *   Z = forward out of the lens (distance to the tag)
	 */
	public static final class VisionConstants {
		// --- Limelight Names (NetworkTables: "limelight-" + name) ---
		// Empty until a camera is mounted on this robot; with no names
		// configured the entire vision stack is inert.
		public static final String[] LIMELIGHT_NAMES = {};

		// --- Pipelines ---
		public static final int APRILTAG_PIPELINE = 0; // Pipeline index used for AprilTag detection

		// --- Camera Mounting ---
		// Facing sign per Limelight, same order as LIMELIGHT_NAMES:
		// +1 = camera faces the robot's FRONT, -1 = faces the REAR. The
		// tracker mirrors its drive commands for rear-facing cameras.
		public static final double[] LIMELIGHT_FACING_SIGNS = {};

		// --- Tag Classes (2025 Reefscape field) ---
		// Only reef and coral station tags are tracked; barge (4, 5, 14, 15)
		// and processor (3, 16) tags are intentionally left blank - the
		// tracker ignores them entirely.
		public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
		public static final int[] CORAL_STATION_TAGS = {1, 2, 12, 13};

		// --- Tracking Goal Distances (meters, camera-space Z) ---
		// All "flush" distances are what the camera READS in that condition,
		// not a field dimension. TUNE by physically placing the robot in the
		// goal position and copying the Vision/Distance dashboard value.
		public static final double REEF_FLUSH_DISTANCE = 0.45;    // Bumpers flush with the reef base (KitBot L1 eject)
		public static final double STATION_FLUSH_DISTANCE = 0.45; // Bumpers flush with the coral station wall

		// --- Tracking Gains and Limits ---
		public static final class TrackingGains {
			public static final double DISTANCE_kP = 1.5;  // TUNE - m/s of drive per meter of position error
			public static final double ROTATION_kP = 0.06; // TUNE - rad/s of rotation per DEGREE of tx error

			public static final double POSITION_ERROR_DEADBAND = 0.05; // Meters; errors below this are treated as zero
			public static final double ROTATION_ERROR_DEADBAND = 1.0;  // Degrees of tx; errors below this are treated as zero

			public static final double MAX_LINEAR_VELOCITY = 2.0;  // m/s command clamp while tracking
			public static final double MAX_ANGULAR_VELOCITY = 1.0; // rad/s command clamp while tracking
		}
	}
}
