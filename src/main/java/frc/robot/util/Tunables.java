package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Live-tunable "magic numbers", backed by WPILib {@link Preferences}.
 *
 * Every value here can be edited from the dashboard without changing or
 * redeploying code: Preferences live in NetworkTables under /Preferences,
 * Elastic's "Robot Preferences" widget (Testing tab) edits them in place, and
 * the roboRIO persists them to /home/lvuser/networktables.json - so an edit
 * survives reboots, power cycles, and future code deploys (a deploy never
 * touches that file). The values in Constants are only the factory defaults,
 * seeded the first time the code runs (or after "Reset Tunables").
 *
 * What belongs here: numbers that get dialed in on the practice field with
 * the robot in front of you - the teleop speed scale, the two flush
 * distances the alignment drives to, and the alignment servo's gains.
 * What does not belong here: the AlLow pivot's Spark MAX gains and preset
 * angles. They are written to the controller once at boot and tuned live in
 * the REV Hardware Client, then copied into Constants; and the AlLow
 * conversion factor is a physical fact to VERIFY, not a knob.
 *
 * Readers call the getters every time they need a value (a Preferences read
 * is one NetworkTables entry lookup - cheap), so edits take effect on the
 * next loop. Every getter clamps to a sane range so a typo on the dashboard
 * cannot command something dangerous.
 */
public final class Tunables {
    private Tunables() {}

    // ------------------------------------------------------------------
    // Keys as shown in the Robot Preferences widget (grouped by prefix)
    // ------------------------------------------------------------------
    private static final String TELEOP_SPEED_SCALE = "Drive - Teleop Speed Scale (0-1)";
    private static final String REEF_FLUSH_DISTANCE = "Vision - Reef Flush Distance (m)";
    private static final String STATION_FLUSH_DISTANCE = "Vision - Station Flush Distance (m)";
    private static final String TRACKING_DISTANCE_KP = "Vision - Tracking Distance kP (m/s per m)";
    private static final String TRACKING_ROTATION_KP = "Vision - Tracking Rotation kP (rad/s per deg)";
    private static final String DEFAULTS_VERSION_KEY = "Tunables - Defaults Version (do not edit)";

    /**
     * Version stamp of the factory defaults in Constants. Stored values
     * survive deploys, so changing a default in Constants does nothing on a
     * robot that already has the key stored - unless this number is bumped.
     * On the first boot after a bump, init() does one of two things:
     *   - if it has a targeted migration block for the stored version, it
     *     overwrites only the keys named there and keeps everything else
     *     the team has tuned on the dashboard;
     *   - otherwise it overwrites every tunable with the new defaults.
     * Bump it when a default changes and must take effect on the robot
     * (and add a migration block when only a few defaults moved); leave it
     * alone to preserve values tuned on the dashboard.
     *
     * Version 1 is the first set of defaults.
     */
    private static final int DEFAULTS_VERSION = 1;

    /**
     * Seeds every key with its Constants default if it does not exist yet
     * (never overwrites a value the team has already tuned). When
     * {@link #DEFAULTS_VERSION} has been bumped since the last boot it
     * instead runs the targeted migration for the stored version or, if
     * there is none, overwrites every key. Call once at robot startup,
     * before the subsystems are constructed.
     */
    public static void init() {
        // A targeted migration goes here, before the version check, when a
        // bump moves only a few defaults - it overwrites only those keys and
        // stamps the new version, so everything else tuned on the dashboard
        // survives. The shape, for a bump from 1 to 2 that only moves the
        // reef flush distance:
        //
        //   if (Preferences.getInt(DEFAULTS_VERSION_KEY, 0) == 1) {
        //       Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        //       Preferences.setInt(DEFAULTS_VERSION_KEY, 2);
        //   }
        if (Preferences.getInt(DEFAULTS_VERSION_KEY, 0) != DEFAULTS_VERSION) {
            resetToDefaults();
            return;
        }
        Preferences.initDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.initDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.initDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.initDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.initDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

    /**
     * Overwrites every tunable with its Constants default and stamps the
     * current defaults version (the "Reset Tunables" dashboard button, and
     * a defaults-version bump at boot that has no targeted migration).
     */
    public static void resetToDefaults() {
        Preferences.setInt(DEFAULTS_VERSION_KEY, DEFAULTS_VERSION);
        Preferences.setDouble(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE);
        Preferences.setDouble(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE);
        Preferences.setDouble(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE);
        Preferences.setDouble(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP);
        Preferences.setDouble(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP);
    }

    /**
     * Like {@link #clamped} but on the magnitude: the vision goal readouts
     * are signed (a tag behind the robot reads negative), and a pasted
     * negative value must not invert a goal.
     */
    private static double clampedMagnitude(String key, double defaultValue, double min, double max) {
        double value = Math.abs(Preferences.getDouble(key, defaultValue));
        return Math.max(min, Math.min(max, value));
    }

    /** Reads a key, falling back to its default, and clamps the result to [min, max]. */
    private static double clamped(String key, double defaultValue, double min, double max) {
        double value = Preferences.getDouble(key, defaultValue);
        return Math.max(min, Math.min(max, value));
    }

    // ------------------------------------------------------------------
    // Driving
    // ------------------------------------------------------------------

    /**
     * Fraction of theoretical top speed / rotation rate at full stick,
     * clamped to [0.05, 1.0] so a bad dashboard entry can neither disable
     * driving nor exceed the drivetrain's capability.
     */
    public static double teleopSpeedScale() {
        return clamped(TELEOP_SPEED_SCALE, DriveConstants.TELEOP_SPEED_SCALE, 0.05, 1.0);
    }

    // ------------------------------------------------------------------
    // Vision alignment goals (meters, robot frame: how far the tag sits
    // from the robot center with the bumper flush)
    // ------------------------------------------------------------------

    /** Distance from the robot center to a reef tag with the rear bumper flush on the reef base. */
    public static double reefFlushDistance() {
        return clampedMagnitude(REEF_FLUSH_DISTANCE, VisionConstants.REEF_FLUSH_DISTANCE, 0.3, 2.0);
    }

    /** Distance from the robot center to a coral-station tag with the front bumper flush on the station wall. */
    public static double stationFlushDistance() {
        return clampedMagnitude(STATION_FLUSH_DISTANCE, VisionConstants.STATION_FLUSH_DISTANCE, 0.3, 2.0);
    }

    // ------------------------------------------------------------------
    // Vision tracking gains
    // ------------------------------------------------------------------

    /** m/s of drive command per meter of position error, clamped to 0 to 3x the default. */
    public static double trackingDistanceKp() {
        return clamped(TRACKING_DISTANCE_KP, VisionConstants.TrackingGains.DISTANCE_kP,
            0.0, 3.0 * VisionConstants.TrackingGains.DISTANCE_kP);
    }

    /** rad/s of rotation command per degree of angle error, clamped to 0 to 3x the default. */
    public static double trackingRotationKp() {
        return clamped(TRACKING_ROTATION_KP, VisionConstants.TrackingGains.ROTATION_kP,
            0.0, 3.0 * VisionConstants.TrackingGains.ROTATION_kP);
    }
}
