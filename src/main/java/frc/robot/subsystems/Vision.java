package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraPose;
import frc.robot.Constants.VisionConstants.TagClass;
import frc.robot.util.Tunables;

/**
 * Vision subsystem: Limelight AprilTag targeting and pose estimation.
 *
 * This robot currently has no Limelights mounted, so
 * VisionConstants.LIMELIGHT_NAMES is empty and this subsystem is inert:
 * periodic() returns at once, nothing touches NetworkTables, and every
 * getter reports "nothing seen". Mounting a camera and filling in the three
 * per-camera arrays in VisionConstants is all it takes to enable it. The
 * stack is the A robot's (same chassis), with the goal resolver reduced to
 * what this robot scores with.
 *
 * Pose estimation: while enabled the robot's heading is sent to each
 * Limelight every loop and the returned MegaTag2 poses are fused into the
 * swerve pose estimator with distance/tag-count based confidence. While
 * disabled (and for a short window after {@link #requestHeadingReseed()}
 * is called) MegaTag1 is fused instead, because its solve carries an absolute
 * heading from tag geometry - that is what seeds the heading MegaTag2 then
 * depends on. Every camera frame is fused at most once (the NT sample
 * timestamp identifies a frame), estimates are inserted oldest first so a
 * late camera cannot erase an earlier camera's correction, single-tag
 * MegaTag1 solves are gated on ambiguity and distance before their heading
 * is trusted, and a camera whose mounting pose is not measured never seeds
 * the heading.
 *
 * Targeting: each camera's primary tag is converted from Limelight camera
 * space into the robot frame using that camera's mounting pose (lens
 * offset, yaw and pitch), so the tracking command servos on where the tag
 * is relative to the robot's center, whichever camera saw it. The tag's
 * field heading (from the AprilTag field layout) gives the heading at which
 * the robot is square to the tag's face. A camera only supplies the tag
 * classes it is mounted for (LIMELIGHT_TRACKING_CLASSES), and only once its
 * lens pose is measured. Which end of the robot is driven up to the tag is
 * a property of the mechanisms, not of the camera: the KitBot roller ejects
 * out of the rear, so a reef tag is backed up to, and the coral is loaded at
 * the front, so a station tag is approached nose-first
 * (VisionConstants.REEF_APPROACH_REAR / STATION_APPROACH_REAR). A camera can
 * therefore only supply a class whose approach side it looks out of. Once
 * tracking starts the first chosen tag is latched so the goal cannot flip
 * between adjacent reef faces mid-approach.
 *
 * What is aligned on is the driver's choice, not inferred: the KitBot has
 * no piece sensor, so the driver holds one button for the reef and another
 * for the coral station ({@link #setAlignmentClass}). Tags of the other
 * class are ignored while that button is held, so a station tag in view
 * cannot hijack a reef alignment. Barge and processor tags have no class
 * and are never aligned on.
 *
 * The tracker never drives on a raw camera solve. A tag does not move, so
 * each new camera frame is turned into a field position for the latched tag
 * - using the robot's pose at the moment the image was captured, not now -
 * and low-pass filtered there; every loop the tracker then gets that field
 * position seen from the robot's current odometry pose. Filtering a static
 * point adds no lag to the control, the 25-100 ms camera latency drops out
 * (driven on directly, it appears as a phantom lateral error whenever the
 * robot is turning: range x the heading change during the latency),
 * single-frame solve noise does not reach the wheels, and a dropped frame
 * changes nothing - which is also how the approach finishes when the
 * latched tag leaves the camera's view in the last stretch.
 *
 * The heading the tracker squares up to is handled the same way, and for
 * the same reason it must not depend on the pose estimator's heading being
 * field-true. The tag's square heading comes from the field layout, but the
 * pose heading is whatever frame the gyro was last zeroed in (a boot
 * orientation, a half-converged seed) - compare the two directly and the
 * robot squares up to the wrong direction, that is, turns away from the tag. So
 * each frame's MegaTag1 solve (which carries a field-true heading from the
 * tag geometry alone) is compared with the pose heading at capture, the
 * difference is filtered, and the square heading is handed to the tracker
 * already converted into the pose estimator's own frame. Nothing here ever
 * writes the pose heading.
 *
 * Latch rules: the latch is released when tracking is switched off, when
 * the latched tag has been out of view for TARGET_MEMORY_SECONDS, or at
 * once when the alignment class changes to another tag family; after that
 * last case nothing new is latched until tracking is switched off and on
 * again, so a changed goal can never send the robot somewhere else under a
 * held button.
 *
 * Separately from alignment, {@link #getClosestSeenTag()} and
 * {@link #getSeenTagsSummary()} report every tag every camera sees, with
 * nothing filtered, so the dashboard agrees with the camera streams.
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class Vision extends SubsystemBase {

    /**
     * A resolved alignment goal: where the tag should sit in the robot frame
     * (WPILib: +X forward, +Y left, meters) when the robot is in position.
     */
    public static class TrackingGoal {
        /** Desired robot-frame X of the tag; negative for a rear (backed-up) approach. */
        public final double forward;
        /** Desired robot-frame Y of the tag (+ = tag to the robot's left). */
        public final double left;

        public TrackingGoal(double forward, double left) {
            this.forward = forward;
            this.left = left;
        }
    }

    /**
     * An AprilTag target: seen by a Limelight this loop, or (fromMemory) the
     * latched tag's last sighting carried on odometry. The camera-space
     * numbers are kept for display; the driving math uses
     * {@link #robotFrame} and {@link #squareHeading}.
     */
    public static class AprilTagTarget {
        public final int id;
        public final TagClass tagClass;
        public final String limelightName;
        public final int cameraIndex;
        public final double tx; // Horizontal angle to target (degrees, + = right), display only
        public final double ty; // Vertical angle to target (degrees), display only
        /** Raw Limelight camera-space position: X right, Y down, Z forward (meters). */
        public final double cameraX, cameraY, cameraZ;
        /** Tag position in the robot frame (+X forward, +Y left), meters. */
        public final Translation2d robotFrame;
        /** True when the robot backs up to this tag (its class is approached rear-first). */
        public final boolean rearApproach;
        /** Field heading at which the robot is square to this tag's face (empty if the tag is not in the layout). */
        public final Optional<Rotation2d> squareHeading;
        /** True when no camera sees the tag and this is its last sighting carried on odometry. */
        public final boolean fromMemory;

        public AprilTagTarget(int id, TagClass tagClass, String limelightName, int cameraIndex,
                double tx, double ty, double cameraX, double cameraY, double cameraZ,
                Translation2d robotFrame, boolean rearApproach, Optional<Rotation2d> squareHeading,
                boolean fromMemory) {
            this.id = id;
            this.tagClass = tagClass;
            this.limelightName = limelightName;
            this.cameraIndex = cameraIndex;
            this.tx = tx;
            this.ty = ty;
            this.cameraX = cameraX;
            this.cameraY = cameraY;
            this.cameraZ = cameraZ;
            this.robotFrame = robotFrame;
            this.rearApproach = rearApproach;
            this.squareHeading = squareHeading;
            this.fromMemory = fromMemory;
        }

        /** Ground-plane distance from the robot center to the tag, in meters. */
        public double distance() {
            return robotFrame.getNorm();
        }
    }

    private final Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;

    /** The tag class the driver is aligning on (set by the hold-to-align bindings); NONE = nothing. */
    private TagClass alignmentClass = TagClass.NONE;

    // Target caches, refreshed once per periodic()
    private Optional<AprilTagTarget> cachedBestTarget = Optional.empty();   // matches the alignment class (drives the tracker)
    private Optional<AprilTagTarget> cachedBestVisible = Optional.empty();  // closest tag a camera may align on (robot frame)
    private Optional<SeenTag> cachedClosestSeen = Optional.empty();         // closest tag any camera reports (dashboard)
    private String cachedSeenTagsSummary = "";

    // Tag latch while tracking: -1 = none. The latched tag's last sighting is
    // remembered as a field position so odometry can carry it while unseen.
    private int latchedTagId = -1;
    private double latchedLastSeenTime = 0.0;
    private TagClass latchedTagClass = TagClass.NONE;
    private boolean latchedRearApproach = false;
    private Optional<Rotation2d> latchedSquareHeading = Optional.empty();
    /** Filtered field position of the latched tag (see the class note); null until its first sample. */
    private Translation2d latchedTagFieldPosition = null;
    /** NT timestamp (microseconds) of the last camera sample folded into the filter: one update per frame. */
    private long latchedSampleStamp = Long.MIN_VALUE;
    /** Consecutive samples that disagreed with the filter by more than the outlier distance. */
    private int latchedOutlierCount = 0;
    /** Filtered (field-true heading - pose estimator heading) while latched; null until the first MegaTag1 sample. */
    private Rotation2d latchedHeadingOffset = null;
    private int latchedHeadingOutlierCount = 0;
    /** True once this tracking session's latch was dropped because the alignment class changed (cleared when tracking is switched off). */
    private boolean latchSpent = false;

    // Precomputed "limelight-<name>" NT table names (avoids per-loop string
    // concatenation in the hot paths). The "limelight-" prefix is the
    // camera's own naming, not a setting.
    private final String[] limelightTableNames;

    // Per-camera fusion bookkeeping (same indexing as LIMELIGHT_NAMES)
    private final double[] lastSeenTimestamp;      // NT timestamp of the last frame examined (dedupe)
    private final Pose2d[] lastFusedPose;          // last estimate fused from each camera (display)
    private final double[] lastFusedTime;          // FPGA time it was fused

    // Heading seed bookkeeping
    private double lastHeadingSeedTime = -1e9;        // FPGA time any trusted MegaTag1 solve was fused
    private double lastStrongHeadingSeedTime = -1e9;  // FPGA time a multi-tag MegaTag1 solve was fused
    private Rotation2d lastStrongSeedHeading = null;  // that solve's own heading
    private double reseedUntil = -1.0;                // FPGA time until which MegaTag1 is fused while enabled

    // 2025 field layout (blue-alliance origin, matching the wpiBlue poses);
    // null with no camera configured (nothing needs it) or if it fails to load
    private final AprilTagFieldLayout fieldLayout;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        int n = VisionConstants.LIMELIGHT_NAMES.length;
        // The three per-camera arrays are indexed together everywhere below:
        // a mismatch is a constants mistake, reported here by name instead of
        // as an ArrayIndexOutOfBounds in the robot loop.
        if (VisionConstants.LIMELIGHT_POSES.length != n || VisionConstants.LIMELIGHT_TRACKING_CLASSES.length != n) {
            throw new IllegalStateException("VisionConstants: LIMELIGHT_NAMES, LIMELIGHT_TRACKING_CLASSES and "
                + "LIMELIGHT_POSES must have one entry per camera, in the same order");
        }
        // A class listed on a camera that faces away from that class's
        // approach side can never finish an approach (the tag leaves the
        // image as the robot squares up), so cameraSupplies() ignores it;
        // say so once, by name.
        for (int i = 0; i < n; i++) {
            for (TagClass listed : VisionConstants.LIMELIGHT_TRACKING_CLASSES[i]) {
                if (listed != TagClass.NONE
                        && VisionConstants.LIMELIGHT_POSES[i].facesRear() != approachesRear(listed)) {
                    DriverStation.reportWarning("VisionConstants: camera '" + VisionConstants.LIMELIGHT_NAMES[i]
                        + "' lists " + listed + " but faces away from the side " + listed
                        + " is approached from; it will not supply that class to alignment", false);
                }
            }
        }
        limelightTableNames = new String[n];
        lastSeenTimestamp = new double[n];
        lastFusedPose = new Pose2d[n];
        lastFusedTime = new double[n];
        for (int i = 0; i < n; i++) {
            limelightTableNames[i] = "limelight-" + VisionConstants.LIMELIGHT_NAMES[i];
            lastSeenTimestamp[i] = Double.NaN;
            lastFusedTime[i] = -1e9;
        }

        AprilTagFieldLayout layout = null;
        if (n > 0) {
            try {
                layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);
            } catch (Exception ex) {
                DriverStation.reportError("Failed to load the AprilTag field layout; alignment falls back to tag bearing",
                    ex.getStackTrace());
            }
        }
        fieldLayout = layout;

        for (int i = 0; i < limelightTableNames.length; i++) {
            String tableName = limelightTableNames[i];
            // Set all Limelights to the AprilTag pipeline
            LimelightHelpers.setPipelineIndex(tableName, VisionConstants.APRILTAG_PIPELINE);
            // LEDs stay off permanently: AprilTags are detected in ambient
            // light and gain nothing from illumination, while the LED array
            // is the camera's single largest heat source (fan noise).
            LimelightHelpers.setLEDMode_ForceOff(tableName);
            // Push the measured lens position/orientation so MegaTag's
            // robot pose is computed from the right camera offset (unmeasured
            // cameras keep their web-UI configuration).
            CameraPose pose = VisionConstants.LIMELIGHT_POSES[i];
            if (pose.measured) {
                LimelightHelpers.setCameraPose_RobotSpace(tableName,
                    pose.forwardMeters, pose.sideMeters, pose.upMeters,
                    pose.rollDegrees, pose.pitchDegrees, pose.yawDegrees);
            }
            LimelightHelpers.setPriorityTagID(tableName, -1);
        }
    }

    /** True when at least one Limelight is configured in VisionConstants (false = the whole stack is inert). */
    public static boolean hasCameras() {
        return VisionConstants.LIMELIGHT_NAMES.length > 0;
    }

    // ------------------------------------------------------------------
    // Geometry (static, unit-tested)
    // ------------------------------------------------------------------

    /**
     * Converts a Limelight camera-space tag position (X right, Y down,
     * Z forward along the optical axis, meters) into the robot frame
     * (+X forward, +Y left) using the camera's mounting pose. The pitch
     * tilts the optical axis, so the ground-plane distance along the
     * camera's heading is Z*cos(pitch) + Y*sin(pitch); the yaw and the lens
     * offset then place the point on the robot.
     */
    public static Translation2d tagPositionInRobotFrame(CameraPose camera, double cameraX, double cameraY, double cameraZ) {
        double pitch = Math.toRadians(camera.pitchDegrees);
        double forwardAlongCameraHeading = cameraZ * Math.cos(pitch) + cameraY * Math.sin(pitch);
        double leftOfCameraHeading = -cameraX;
        Translation2d inCameraHeadingFrame = new Translation2d(forwardAlongCameraHeading, leftOfCameraHeading)
            .rotateBy(Rotation2d.fromDegrees(camera.yawDegrees));
        return camera.lensOnRobot().plus(inCameraHeadingFrame);
    }

    /** A robot-frame point (+X forward, +Y left) expressed as a field position, given the robot's field pose. */
    public static Translation2d robotFrameToField(Pose2d robot, Translation2d inRobotFrame) {
        return robot.getTranslation().plus(inRobotFrame.rotateBy(robot.getRotation()));
    }

    /** A field position expressed in the robot frame (+X forward, +Y left), given the robot's field pose. */
    public static Translation2d fieldToRobotFrame(Pose2d robot, Translation2d onField) {
        return onField.minus(robot.getTranslation()).rotateBy(robot.getRotation().unaryMinus());
    }

    /**
     * Field heading at which the robot is square to the given tag's face:
     * facing it for a front approach (the robot looks the opposite way to
     * the tag), backed up to it for a rear approach (the robot looks the
     * same way the tag does).
     */
    public static Optional<Rotation2d> squareHeading(AprilTagFieldLayout layout, int tagId, boolean rearApproach) {
        if (layout == null) {
            return Optional.empty();
        }
        return layout.getTagPose(tagId).map(pose -> {
            Rotation2d tagFacing = pose.toPose2d().getRotation();
            return rearApproach ? tagFacing : tagFacing.plus(Rotation2d.k180deg);
        });
    }

    /** The class of a 2025 Reefscape tag: reef, coral station, or NONE (barge / processor / unknown). */
    public static TagClass classOf(int tagId) {
        for (int tag : VisionConstants.REEF_TAGS) {
            if (tag == tagId) {
                return TagClass.REEF;
            }
        }
        for (int tag : VisionConstants.CORAL_STATION_TAGS) {
            if (tag == tagId) {
                return TagClass.CORAL_STATION;
            }
        }
        return TagClass.NONE;
    }

    /**
     * True when tags of this class are approached rear-first (backed up
     * to). It follows from where the mechanisms are: the KitBot roller
     * ejects out of the rear, the coral is loaded at the front.
     */
    public static boolean approachesRear(TagClass tagClass) {
        return tagClass == TagClass.REEF
            ? VisionConstants.REEF_APPROACH_REAR
            : VisionConstants.STATION_APPROACH_REAR;
    }

    /**
     * The alignment goal for a tag of the given class: bumper flush and
     * centered on the tag, at the end of the robot that class is approached
     * from ({@link #approachesRear}).
     *
     *  - Reef: rear bumper flush with the reef base and centered on the
     *    face - the KitBot roller ejects into the L1 trough from there, and
     *    the trough runs the width of the face, so there is no left / right
     *    branch.
     *  - Coral station: front bumper flush with the station wall, centered
     *    on the tag.
     *  - NONE (barge, processor, unknown): no goal.
     *
     * Goals are in the robot frame: forward = where the tag should be along
     * +X (negative = behind the robot center, that is, a backed-up approach),
     * left = where it should be along +Y (0 = centered; REEF_GOAL_LEFT_METERS
     * and STATION_GOAL_LEFT_METERS).
     */
    public static Optional<TrackingGoal> trackingGoal(TagClass tagClass,
            double reefFlushDistance, double stationFlushDistance) {
        double side = approachesRear(tagClass) ? -1.0 : 1.0;
        switch (tagClass) {
            case REEF:
                return Optional.of(new TrackingGoal(side * reefFlushDistance, VisionConstants.REEF_GOAL_LEFT_METERS));
            case CORAL_STATION:
                return Optional.of(new TrackingGoal(side * stationFlushDistance, VisionConstants.STATION_GOAL_LEFT_METERS));
            default:
                return Optional.empty();
        }
    }

    // ------------------------------------------------------------------
    // Throttle
    // ------------------------------------------------------------------

    /**
     * Frame throttle currently applied to the cameras (-1 = not yet sent).
     * Tracked so the NT write happens only when the throttle changes.
     */
    private int appliedThrottle = -1;

    /**
     * Throttles AprilTag processing while the robot is disabled and restores
     * full rate when enabled - the cameras spend most of their powered-on
     * life disabled in the pit, where full-rate processing only makes heat.
     * Selecting Test mode on the Driver Station lifts the throttle without
     * enabling: throttled, every vision readout on the dashboard is a second
     * or more behind the camera's own stream, too slow for checking a camera
     * on the bench.
     */
    private void updateThrottle() {
        int throttle = DriverStation.isDisabled() && !DriverStation.isTest()
            ? VisionConstants.DISABLED_THROTTLE
            : VisionConstants.ENABLED_THROTTLE;
        if (throttle != appliedThrottle) {
            for (String tableName : limelightTableNames) {
                LimelightHelpers.SetThrottle(tableName, throttle);
            }
            appliedThrottle = throttle;
        }
    }

    // ------------------------------------------------------------------
    // Tracking state
    // ------------------------------------------------------------------

    /** Whether the Limelight at the given LIMELIGHT_NAMES index sees a target. */
    public boolean hasTarget(int index) {
        return LimelightHelpers.getTV(limelightTableNames[index]);
    }

    /**
     * Records whether AprilTag tracking is active. Deliberately does not
     * touch the LEDs: they add nothing to AprilTag detection and are the
     * camera's largest heat source (and the cause of its fan noise).
     * Turning tracking off releases the tag latch, re-arms latching and
     * clears the alignment class, so the next alignment starts clean even
     * when it begins in the same loop this one ended in (the driver rolling
     * from one align trigger onto the other).
     */
    public void setTrackingEnabled(boolean enabled) {
        trackingEnabled = enabled;
        if (!enabled) {
            releaseLatch();
            latchSpent = false;
            alignmentClass = TagClass.NONE;
        }
    }

    /**
     * Whether AprilTag tracking is enabled. Intentionally unused: kept as
     * the getter paired with setTrackingEnabled() for dashboards and tests
     * (Swerve keeps its own copy of this state for the align bindings).
     */
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    /**
     * Selects what the tracker aligns on: REEF or CORAL_STATION (NONE =
     * nothing). Set by the driver's hold-to-align bindings immediately before
     * tracking is switched on; switching tracking off sets it back to NONE.
     */
    public void setAlignmentClass(TagClass tagClass) {
        this.alignmentClass = tagClass;
    }

    public TagClass getAlignmentClass() {
        return alignmentClass;
    }

    /** The tag id the tracker is latched onto, or -1. */
    public int getLatchedTagId() {
        return latchedTagId;
    }

    /**
     * Resolves the alignment goal for a target from its tag class and the
     * live flush-distance tunables - see {@link #trackingGoal}.
     *
     * @return the goal, or empty for a tag class that has none
     */
    public Optional<TrackingGoal> getTrackingGoal(AprilTagTarget target) {
        return trackingGoal(target.tagClass,
            Tunables.reefFlushDistance(), Tunables.stationFlushDistance());
    }

    /** The target the tracker should drive to this loop (cached; refreshed in periodic). */
    public Optional<AprilTagTarget> getBestTarget() {
        return cachedBestTarget;
    }

    /**
     * The closest tag a camera may align on this loop (its class is one the
     * camera is mounted for, and the camera's lens pose is measured),
     * regardless of the alignment class or the latch. It carries a
     * robot-frame position, so it feeds the distance / lateral readouts used
     * to measure the flush distances with the robot pushed into position.
     * For "which tag do the cameras see" use {@link #getClosestSeenTag()},
     * which filters nothing.
     */
    public Optional<AprilTagTarget> getBestVisibleTarget() {
        return cachedBestVisible;
    }

    // ------------------------------------------------------------------
    // What the cameras see (dashboard)
    // ------------------------------------------------------------------

    /**
     * One tag exactly as a camera reports it, with nothing filtered: no tag
     * class, no camera role, no mounting pose, no latch. This is what the
     * camera's own stream draws, which is what a "which tag do you see"
     * readout has to agree with. The alignment caches above are a strict
     * subset of it - a camera only aligns on the classes it is mounted for,
     * and only once its lens pose is measured.
     */
    public static final class SeenTag {
        public final int id;
        public final int cameraIndex;
        /** Camera to tag, meters; 0 when the camera has no 3D solve for it. */
        public final double distanceMeters;
        /** Share of the image the tag covers, percent. */
        public final double area;

        SeenTag(int id, int cameraIndex, double distanceMeters, double area) {
            this.id = id;
            this.cameraIndex = cameraIndex;
            this.distanceMeters = distanceMeters;
            this.area = area;
        }
    }

    /**
     * Values per tag in a Limelight "rawfiducials" array: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity.
     * Limelight's array format, not a setting, so it stays here.
     */
    private static final int RAW_FIDUCIAL_STRIDE = 7;

    /**
     * The closest tag in the cameras' "rawfiducials" arrays (one per camera,
     * null or empty = that camera reports nothing). Closest by the 3D
     * distance when any tag has one; by image area (largest) when none does,
     * so the readout still works on a pipeline with the 3D solve off.
     */
    public static Optional<SeenTag> closestSeenTag(double[][] rawFiducialsByCamera) {
        SeenTag byDistance = null;
        SeenTag byArea = null;
        for (int camera = 0; camera < rawFiducialsByCamera.length; camera++) {
            double[] raw = rawFiducialsByCamera[camera];
            if (raw == null || raw.length % RAW_FIDUCIAL_STRIDE != 0) {
                continue;
            }
            for (int base = 0; base < raw.length; base += RAW_FIDUCIAL_STRIDE) {
                SeenTag tag = new SeenTag((int) raw[base], camera, raw[base + 4], raw[base + 3]);
                if (tag.distanceMeters > 0 && (byDistance == null || tag.distanceMeters < byDistance.distanceMeters)) {
                    byDistance = tag;
                }
                if (byArea == null || tag.area > byArea.area) {
                    byArea = tag;
                }
            }
        }
        return Optional.ofNullable(byDistance != null ? byDistance : byArea);
    }

    /** Every tag ID per camera, in the cameras' own order: "front: 18 19 | rear: 12" ("" = none). */
    public static String seenTagsSummary(String[] cameraNames, double[][] rawFiducialsByCamera) {
        StringBuilder summary = new StringBuilder();
        for (int camera = 0; camera < rawFiducialsByCamera.length; camera++) {
            double[] raw = rawFiducialsByCamera[camera];
            if (raw == null || raw.length == 0 || raw.length % RAW_FIDUCIAL_STRIDE != 0) {
                continue;
            }
            if (summary.length() > 0) {
                summary.append(" | ");
            }
            summary.append(cameraNames[camera]).append(':');
            for (int base = 0; base < raw.length; base += RAW_FIDUCIAL_STRIDE) {
                summary.append(' ').append((int) raw[base]);
            }
        }
        return summary.toString();
    }

    private final double[][] rawFiducialsByCamera = new double[VisionConstants.LIMELIGHT_NAMES.length][];

    /**
     * Reads what every camera reports this loop. A camera whose array has
     * not changed for SEEN_TAG_STALE_SECONDS is treated as reporting nothing:
     * NetworkTables keeps the last value of a camera that lost power or its
     * link, and a tag in view never yields two identical arrays in a row.
     */
    private void refreshSeenTags() {
        double now = Timer.getFPGATimestamp();
        for (int i = 0; i < limelightTableNames.length; i++) {
            var sample = LimelightHelpers.getLimelightDoubleArrayEntry(limelightTableNames[i], "rawfiducials").getAtomic();
            boolean stale = now - sample.timestamp / 1e6 > VisionConstants.SEEN_TAG_STALE_SECONDS; // NT time is in microseconds
            rawFiducialsByCamera[i] = stale ? null : sample.value;
        }
        cachedClosestSeen = closestSeenTag(rawFiducialsByCamera);
        cachedSeenTagsSummary = seenTagsSummary(VisionConstants.LIMELIGHT_NAMES, rawFiducialsByCamera);
    }

    /** The closest tag any camera reports this loop, unfiltered - what the dashboard's "Best Tag" shows. */
    public Optional<SeenTag> getClosestSeenTag() {
        return cachedClosestSeen;
    }

    /** Every tag ID each camera reports this loop ("front: 18 19 | rear: 12"), "" when none. */
    public String getSeenTagsSummary() {
        return cachedSeenTagsSummary;
    }

    /**
     * True when the tag's class is the one the driver is aligning on.
     * Without this, a station tag seen at 3 m could out-"close" the intended
     * reef tag at 4 m and hijack a scoring alignment.
     */
    private boolean tagMatchesAlignmentClass(TagClass tagClass) {
        return tagClass != TagClass.NONE && tagClass == alignmentClass;
    }

    /**
     * True when the camera may supply this tag class to alignment: the class
     * is listed for it, and the camera looks out of the end of the robot
     * that class is approached from.
     */
    private static boolean cameraSupplies(int cameraIndex, TagClass tagClass) {
        if (VisionConstants.LIMELIGHT_POSES[cameraIndex].facesRear() != approachesRear(tagClass)) {
            return false;
        }
        for (TagClass allowed : VisionConstants.LIMELIGHT_TRACKING_CLASSES[cameraIndex]) {
            if (allowed == tagClass) {
                return true;
            }
        }
        return false;
    }

    private void latch(AprilTagTarget target, long sampleStamp, double captureTime, double fieldTrueYawDegrees,
            Pose2d robotPose) {
        latchedTagId = target.id;
        latchedTagClass = target.tagClass;
        latchedRearApproach = target.rearApproach;
        latchedSquareHeading = target.squareHeading;
        latchedTagFieldPosition = null;
        latchedSampleStamp = Long.MIN_VALUE;
        latchedOutlierCount = 0;
        latchedHeadingOffset = null;
        latchedHeadingOutlierCount = 0;
        updateLatchedEstimate(target, sampleStamp, captureTime, fieldTrueYawDegrees, robotPose);
        for (String tableName : limelightTableNames) {
            LimelightHelpers.setPriorityTagID(tableName, target.id);
        }
    }

    /**
     * Folds one sighting of the latched tag into its filtered field
     * position. Only a new camera frame counts (the robot loop reads the
     * same frame several times), and the frame is placed on the field with
     * the pose the robot had when the image was captured. A sample far from
     * the estimate is ignored unless it persists - one bad solve must not
     * yank the goal, but a real change (a pose reset) is followed.
     */
    private void updateLatchedEstimate(AprilTagTarget target, long sampleStamp, double captureTime,
            double fieldTrueYawDegrees, Pose2d robotPose) {
        latchedLastSeenTime = Timer.getFPGATimestamp();
        if (sampleStamp == latchedSampleStamp) {
            return;
        }
        latchedSampleStamp = sampleStamp;
        Pose2d poseAtCapture = swerve.samplePoseAt(Utils.fpgaToCurrentTime(captureTime)).orElse(robotPose);
        updateHeadingOffset(fieldTrueYawDegrees, poseAtCapture.getRotation());
        Translation2d measured = robotFrameToField(poseAtCapture, target.robotFrame);
        if (latchedTagFieldPosition == null) {
            latchedTagFieldPosition = measured;
            return;
        }
        if (measured.getDistance(latchedTagFieldPosition) > VisionConstants.TrackingGains.TARGET_OUTLIER_METERS
                && ++latchedOutlierCount < VisionConstants.TrackingGains.TARGET_OUTLIER_FRAMES) {
            return;
        }
        latchedTagFieldPosition = latchedOutlierCount > 0
            ? measured
            : latchedTagFieldPosition.interpolate(measured, VisionConstants.TrackingGains.TARGET_FILTER_ALPHA);
        latchedOutlierCount = 0;
    }

    /**
     * Folds one frame's field-true heading (MegaTag1, NaN when that frame
     * had no solve) into the filtered offset between the field-true heading
     * and the pose estimator's. A single-tag solve can flip (pose
     * ambiguity), so a sample far from the estimate only counts once it
     * persists.
     */
    private void updateHeadingOffset(double fieldTrueYawDegrees, Rotation2d poseHeadingAtCapture) {
        if (Double.isNaN(fieldTrueYawDegrees)) {
            return;
        }
        Rotation2d measured = Rotation2d.fromDegrees(fieldTrueYawDegrees).minus(poseHeadingAtCapture);
        if (latchedHeadingOffset == null) {
            latchedHeadingOffset = measured;
            return;
        }
        Rotation2d innovation = measured.minus(latchedHeadingOffset);
        if (Math.abs(innovation.getDegrees()) > VisionConstants.TrackingGains.HEADING_OUTLIER_DEGREES
                && ++latchedHeadingOutlierCount < VisionConstants.TrackingGains.HEADING_OUTLIER_FRAMES) {
            return;
        }
        latchedHeadingOffset = latchedHeadingOutlierCount > 0
            ? measured
            : latchedHeadingOffset.plus(innovation.times(VisionConstants.TrackingGains.HEADING_FILTER_ALPHA));
        latchedHeadingOutlierCount = 0;
    }

    /**
     * The latched tag's square heading in the pose estimator's frame (what
     * the tracker compares with the pose heading). Falls back to the layout
     * heading as it is until a MegaTag1 sample has arrived.
     */
    private Optional<Rotation2d> squareHeadingInPoseFrame() {
        if (latchedHeadingOffset == null) {
            return latchedSquareHeading;
        }
        return latchedSquareHeading.map(square -> square.minus(latchedHeadingOffset));
    }

    /** Field-true minus pose-estimator heading while latched, degrees (0 when unknown) - for the dashboard. */
    public double getLatchedHeadingOffsetDegrees() {
        return latchedHeadingOffset == null ? 0.0 : latchedHeadingOffset.getDegrees();
    }

    private void releaseLatch() {
        if (latchedTagId < 0) {
            return;
        }
        latchedTagId = -1;
        latchedTagFieldPosition = null;
        latchedHeadingOffset = null;
        for (String tableName : limelightTableNames) {
            LimelightHelpers.setPriorityTagID(tableName, -1);
        }
    }

    /**
     * The latched tag as the tracker should see it: its filtered field
     * position from the robot's current pose. {@code live} is this loop's
     * camera sighting (its display numbers are kept), or null while the tag
     * is out of view.
     */
    private AprilTagTarget carriedTarget(Pose2d robotPose, AprilTagTarget live) {
        Translation2d inRobot = fieldToRobotFrame(robotPose, latchedTagFieldPosition);
        if (live == null) {
            return new AprilTagTarget(latchedTagId, latchedTagClass, "memory", -1,
                0.0, 0.0, 0.0, 0.0, 0.0, inRobot, latchedRearApproach, squareHeadingInPoseFrame(), true);
        }
        return new AprilTagTarget(live.id, live.tagClass, live.limelightName, live.cameraIndex,
            live.tx, live.ty, live.cameraX, live.cameraY, live.cameraZ, inRobot, latchedRearApproach,
            squareHeadingInPoseFrame(), false);
    }

    /**
     * Queries every Limelight and refreshes both alignment caches: the
     * closest tag a camera may align on, whatever the alignment class (the
     * dashboard's "Alignment Tag" readouts), and the closest tag of the
     * class being aligned on (tracker), honoring the latch while tracking.
     * While the latched tag is unseen (for up to TARGET_MEMORY_SECONDS) its
     * last sighting carried on odometry stands in, so an approach whose tag
     * leaves the camera's view at the end still finishes.
     */
    private void refreshTargets(Pose2d robotPose) {
        double now = Timer.getFPGATimestamp();
        AprilTagTarget bestVisible = null;
        double bestVisibleDistance = Double.MAX_VALUE;
        AprilTagTarget bestForGoal = null;
        double bestGoalDistance = Double.MAX_VALUE;
        long bestGoalStamp = 0;
        double bestGoalCaptureTime = now;
        double bestGoalFieldTrueYaw = Double.NaN;
        boolean latchedSeen = false;

        for (int i = 0; i < limelightTableNames.length; i++) {
            CameraPose cameraPose = VisionConstants.LIMELIGHT_POSES[i];
            if (!cameraPose.measured) {
                continue; // alignment needs the lens pose: an unmeasured camera supplies nothing
            }
            String limelightName = limelightTableNames[i];
            if (!LimelightHelpers.getTV(limelightName)) {
                continue;
            }
            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            TagClass tagClass = classOf(tagId);
            if (tagClass == TagClass.NONE || !cameraSupplies(i, tagClass)) {
                continue; // not a class this camera is mounted for
            }
            // Raw camera-space array: an empty array (no 3D solve / topic
            // absent) or an all-zero one (3D solve disabled) must not become
            // a "0 m away" target that wins the closest-tag selection.
            // Read atomically with its NT timestamp: the timestamp identifies
            // the camera frame, and minus the pipeline + capture latency it
            // is when the image was taken.
            var sample = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "targetpose_cameraspace").getAtomic();
            double[] cameraSpace = sample.value;
            if (cameraSpace.length < 6 || !(cameraSpace[2] > VisionConstants.MIN_CAMERA_Z)) {
                continue;
            }
            Translation2d inRobot = tagPositionInRobotFrame(cameraPose,
                cameraSpace[0], cameraSpace[1], cameraSpace[2]);
            boolean rearApproach = approachesRear(tagClass);
            AprilTagTarget target = new AprilTagTarget(
                tagId, tagClass, VisionConstants.LIMELIGHT_NAMES[i], i,
                LimelightHelpers.getTX(limelightName), LimelightHelpers.getTY(limelightName),
                cameraSpace[0], cameraSpace[1], cameraSpace[2],
                inRobot, rearApproach, squareHeading(fieldLayout, tagId, rearApproach), false);
            double distance = target.distance();
            if (distance < bestVisibleDistance) {
                bestVisible = target;
                bestVisibleDistance = distance;
            }
            if (!tagMatchesAlignmentClass(tagClass)) {
                continue;
            }
            if (latchedTagId >= 0 && tagId != latchedTagId) {
                continue; // latched onto another tag
            }
            if (tagId == latchedTagId) {
                latchedSeen = true;
            }
            if (distance < bestGoalDistance) {
                bestForGoal = target;
                bestGoalDistance = distance;
                bestGoalStamp = sample.timestamp;
                bestGoalCaptureTime = sample.timestamp / 1e6
                    - (LimelightHelpers.getLatency_Pipeline(limelightName)
                        + LimelightHelpers.getLatency_Capture(limelightName)) / 1000.0;
                // MegaTag1 robot pose for the same frame: [x, y, z, roll,
                // pitch, yaw, latency, tag count, ...], blue origin, degrees.
                // Its yaw is field-true from the tag geometry alone. Only a
                // camera with a measured mounting pose can supply it.
                double[] mt1 = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_wpiblue").get();
                bestGoalFieldTrueYaw = mt1.length >= 8 && mt1[7] >= 1 ? mt1[5] : Double.NaN;
            }
        }

        cachedBestVisible = Optional.ofNullable(bestVisible);
        cachedBestTarget = Optional.ofNullable(bestForGoal);

        // Latch management: hold the first chosen tag while tracking, fold
        // each new frame of it into the filtered field position, and drop it
        // once it has been out of view for the memory time. While latched
        // the tracker always gets the filtered position seen from the
        // current odometry pose - seen this loop or not.
        if (!trackingEnabled) {
            releaseLatch();
            latchSpent = false;
            return;
        }
        // The alignment class has left the latched tag's family. Drop the
        // latch now rather than letting it live on in memory for
        // TARGET_MEMORY_SECONDS, and latch nothing new until the driver lets
        // go and presses again: a goal that changes under a held button must
        // not send the robot off to a different field element by itself.
        if (latchedTagId >= 0 && !tagMatchesAlignmentClass(latchedTagClass)) {
            releaseLatch();
            latchSpent = true;
        }
        if (latchSpent) {
            cachedBestTarget = Optional.empty();
            return;
        }
        if (latchedTagId < 0) {
            if (bestForGoal != null) {
                latch(bestForGoal, bestGoalStamp, bestGoalCaptureTime, bestGoalFieldTrueYaw, robotPose);
            }
        } else if (latchedSeen) {
            updateLatchedEstimate(bestForGoal, bestGoalStamp, bestGoalCaptureTime, bestGoalFieldTrueYaw, robotPose);
        } else if (now - latchedLastSeenTime > VisionConstants.TrackingGains.TARGET_MEMORY_SECONDS) {
            releaseLatch();
        }
        if (latchedTagId >= 0 && latchedTagFieldPosition != null) {
            cachedBestTarget = Optional.of(carriedTarget(robotPose, latchedSeen ? bestForGoal : null));
        }
    }

    // ------------------------------------------------------------------
    // Pose fusion
    // ------------------------------------------------------------------

    /**
     * Enables or disables MegaTag pose fusion. Useful during testing to
     * compare pure wheel odometry against vision-corrected odometry.
     *
     * Intentionally unused (with its getter below): fusion defaults
     * to on; call or temporarily bind this in test sessions only.
     */
    public void setPositionTrackingEnabled(boolean enabled) {
        positionTrackingEnabled = enabled;
    }

    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    /**
     * Asks for the heading to be re-seeded from tag geometry: MegaTag1 (whose
     * solve carries an absolute heading) is fused for the next
     * HEADING_RESEED_WINDOW_SECONDS even though the robot is enabled. It
     * corrects a drifted pose heading while a tag is in view; unlike a gyro
     * re-zero it cannot feed MegaTag2 a made-up heading, and with no tag in
     * view it does nothing. Bound to the driver's Y button. (The driver's
     * left bumper is something else: it zeroes the driver's own heading
     * frame - see Swerve.zeroDriverHeading.)
     */
    public void requestHeadingReseed() {
        reseedUntil = Timer.getFPGATimestamp() + VisionConstants.HEADING_RESEED_WINDOW_SECONDS;
    }

    /** True while a driver-requested heading re-seed window is open. */
    public boolean isReseedingHeading() {
        return Timer.getFPGATimestamp() < reseedUntil;
    }

    /**
     * True when a MegaTag1 solve with a trusted heading (multi-tag, or one
     * close unambiguous tag) was fused within
     * HEADING_SEED_FRESHNESS_SECONDS - that is, the pose heading has been
     * pulled toward a field-referenced value recently.
     */
    public boolean hasFreshHeadingSeed() {
        return Timer.getFPGATimestamp() - lastHeadingSeedTime <= VisionConstants.HEADING_SEED_FRESHNESS_SECONDS;
    }

    /**
     * True when the pose heading has converged on a strong seed: a
     * multi-tag MegaTag1 solve (MT1_MULTI_TAG_MIN_COUNT or more tags) was
     * fused within
     * HEADING_SEED_FRESHNESS_SECONDS and the estimator's heading now agrees
     * with that solve's own heading within HEADING_SEED_AGREEMENT_DEGREES.
     * (One fused solve only closes part of the heading error, so freshness
     * alone would trust a heading that is still converging.)
     */
    public boolean hasStrongHeadingSeed() {
        if (lastStrongSeedHeading == null
                || Timer.getFPGATimestamp() - lastStrongHeadingSeedTime > VisionConstants.HEADING_SEED_FRESHNESS_SECONDS) {
            return false;
        }
        Rotation2d current = swerve.getStateCopy().Pose.getRotation();
        return Math.abs(current.minus(lastStrongSeedHeading).getDegrees()) <= VisionConstants.HEADING_SEED_AGREEMENT_DEGREES;
    }

    /** The last pose fused from the given camera, if it was fused within the last FUSED_POSE_DISPLAY_SECONDS. */
    public Optional<Pose2d> getLastFusedPose(int index) {
        if (lastFusedPose[index] == null
                || Timer.getFPGATimestamp() - lastFusedTime[index] > VisionConstants.FUSED_POSE_DISPLAY_SECONDS) {
            return Optional.empty();
        }
        return Optional.of(lastFusedPose[index]);
    }

    /**
     * Rejects estimates that cannot be right: NaN or off-field poses, far
     * MegaTag2 solves, and single-tag MegaTag1 solves that are ambiguous or
     * far (the classic single-tag pose flip would otherwise seed a heading
     * that is wrong by tens of degrees).
     */
    private static boolean isPlausible(PoseEstimate estimate, boolean megaTag1) {
        Translation2d t = estimate.pose.getTranslation();
        if (Double.isNaN(t.getX()) || Double.isNaN(t.getY())) {
            return false;
        }
        double margin = VisionConstants.FIELD_BOUNDS_MARGIN_METERS;
        if (t.getX() < -margin || t.getX() > VisionConstants.FIELD_LENGTH_METERS + margin
                || t.getY() < -margin || t.getY() > VisionConstants.FIELD_WIDTH_METERS + margin) {
            return false;
        }
        if (megaTag1) {
            if (estimate.tagCount < VisionConstants.MT1_MULTI_TAG_MIN_COUNT) { // a single-tag solve: the kind with an ambiguity to gate
                if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
                    return false;
                }
                RawFiducial fiducial = estimate.rawFiducials[0];
                if (fiducial.ambiguity > VisionConstants.MT1_SINGLE_TAG_MAX_AMBIGUITY
                        || fiducial.distToCamera > VisionConstants.MT1_SINGLE_TAG_MAX_DISTANCE_METERS) {
                    return false;
                }
            }
        } else if (estimate.avgTagDist > VisionConstants.MT2_MAX_AVG_TAG_DISTANCE_METERS) {
            return false;
        }
        return true;
    }

    /**
     * Fuses vision pose estimates from every Limelight into the drivetrain
     * odometry. Timestamp conversion to the Phoenix timebase happens inside
     * Swerve.addVisionMeasurement().
     *
     * Two modes:
     *  - Disabled (pre-match / between periods) or a driver re-seed window:
     *    MegaTag1, whose solve includes an absolute heading from tag
     *    geometry alone. Its rotation is fused so the pose heading converges
     *    to field-correct while the robot sits still - without this, the
     *    heading MegaTag2 depends on would start at whatever the gyro booted
     *    to and every fused pose would be wrong until a manual reset. Only
     *    cameras with a measured mounting pose take part: a placeholder
     *    lens pose would seed a biased heading.
     *  - Enabled: MegaTag2, which takes our (now-seeded) heading and returns
     *    a far more stable translation than single-tag solves. Its heading
     *    is our own gyro echoed back, so it gets effectively zero weight.
     *
     * Each camera frame is fused once: the estimator applies its correction
     * on every call, so re-adding the same sample every 20 ms loop (the
     * camera publishes at most about 90 Hz, and only about 1 Hz while throttled) would
     * collapse the estimate onto the raw camera pose regardless of the
     * standard deviations. The batch is inserted oldest first because
     * inserting an older measurement discards the corrections from newer
     * ones. Frames that arrive while the drivetrain is rejecting vision
     * (right after a fast spin) are dropped, not deferred.
     */
    private void updateRobotPosition(Pose2d robotPose) {
        double now = Timer.getFPGATimestamp();
        boolean seedingHeading = DriverStation.isDisabled() || now < reseedUntil;
        boolean rejecting = swerve.isRejectingVision();
        double headingDegrees = robotPose.getRotation().getDegrees();

        List<PoseEstimate> batch = new ArrayList<>(limelightTableNames.length);
        List<Integer> batchCameras = new ArrayList<>(limelightTableNames.length);
        for (int i = 0; i < limelightTableNames.length; i++) {
            String tableName = limelightTableNames[i];
            // NoFlush variant: one NT flush after the loop instead of a full
            // network flush per camera per loop
            LimelightHelpers.SetRobotOrientation_NoFlush(tableName, headingDegrees, 0, 0, 0, 0, 0);

            if (seedingHeading && !VisionConstants.LIMELIGHT_POSES[i].measured) {
                continue; // an unknown lens pose cannot seed the heading (or the translation)
            }

            PoseEstimate estimate = seedingHeading
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tableName);

            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }
            // Same NT sample as last loop: this frame has already been
            // considered (an unchanged sample reads back a bit-identical
            // timestamp; every new frame gets a new server timestamp).
            if (estimate.timestampSeconds == lastSeenTimestamp[i]) {
                continue;
            }
            lastSeenTimestamp[i] = estimate.timestampSeconds;
            if (rejecting || !isPlausible(estimate, seedingHeading)) {
                continue;
            }
            batch.add(estimate);
            batchCameras.add(i);
        }

        // Oldest first, so a late camera cannot erase an earlier camera's correction
        Integer[] order = new Integer[batch.size()];
        for (int k = 0; k < order.length; k++) {
            order[k] = k;
        }
        java.util.Arrays.sort(order, Comparator.comparingDouble(k -> batch.get(k).timestampSeconds));

        for (int k : order) {
            PoseEstimate estimate = batch.get(k);
            int camera = batchCameras.get(k);
            // Confidence scales with tag count and closeness
            double xyStdDev = VisionConstants.XY_STD_DEV_BASE
                + VisionConstants.XY_STD_DEV_DISTANCE_SQUARED_GAIN * (estimate.avgTagDist * estimate.avgTagDist)
                    / Math.max(1, estimate.tagCount);
            // MegaTag1 heading is trusted while seeding - tightly for a
            // multi-tag solve so the stationary heading collapses onto the
            // solve in a few frames even at the disabled throttle's about
            // 1 Hz - and MegaTag2's heading never is.
            double rotStdDev = seedingHeading
                ? (estimate.tagCount >= VisionConstants.MT1_MULTI_TAG_MIN_COUNT
                    ? VisionConstants.MT1_MULTI_TAG_ROTATION_STD_DEV
                    : VisionConstants.MT1_SINGLE_TAG_ROTATION_STD_DEV)
                : VisionConstants.MT2_ROTATION_STD_DEV;

            swerve.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

            lastFusedPose[camera] = estimate.pose;
            lastFusedTime[camera] = now;
            if (seedingHeading) {
                lastHeadingSeedTime = now;
                if (estimate.tagCount >= VisionConstants.MT1_MULTI_TAG_MIN_COUNT) {
                    lastStrongHeadingSeedTime = now;
                    lastStrongSeedHeading = estimate.pose.getRotation();
                }
            }
        }

        LimelightHelpers.Flush();
    }

    @Override
    public void periodic() {
        // No camera configured: nothing to read, nothing to send, and no
        // NetworkTables flush - every cache keeps its empty initial value.
        if (!hasCameras()) {
            return;
        }

        // Full-rate processing only while enabled or in Test mode (thermal / fan noise)
        updateThrottle();

        // One pose snapshot for this loop (getState() is the shared object
        // the odometry thread rewrites)
        Pose2d robotPose = swerve.getStateCopy().Pose;

        // Refresh the target caches once per loop; all readers use these.
        refreshTargets(robotPose);
        refreshSeenTags();

        if (positionTrackingEnabled) {
            updateRobotPosition(robotPose);
        }
    }
}
