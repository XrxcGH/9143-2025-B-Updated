package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

/**
 * Vision subsystem: Limelight AprilTag targeting and pose estimation.
 *
 * NOTE: this robot currently has no Limelights mounted, so
 * VisionConstants.LIMELIGHT_NAMES is empty and every method here is inert.
 * The stack is kept fully wired (ported from the A robot) so that mounting a
 * camera and adding its name to the constants is all it takes to enable it.
 *
 * Pose estimation uses MegaTag2: the robot's gyro heading is sent to each
 * Limelight every loop, and the returned pose estimates are fused into the
 * swerve drivetrain's odometry with distance/tag-count based confidence.
 *
 * The "best target" (closest TRACKABLE tag - reef or coral station; barge
 * and processor tags are ignored) is computed once per loop in periodic()
 * and cached, so dashboard widgets and the tracking command can read it
 * without re-querying NetworkTables dozens of times per cycle.
 *
 * Tracking goals depend only on the tag's class on this robot: the KitBot
 * scores by ejecting into the L1 trough, so reef alignment is always
 * bumpers-flush and centered on the tag (no branch selection or standoff
 * poses like the A robot's superstructure needs).
 *
 * Dashboard note: this subsystem publishes nothing itself. All telemetry is
 * read through the public getters by the central {@link frc.robot.Dashboard}
 * class, which owns every NetworkTables/Elastic publication for the robot.
 */
public class Vision extends SubsystemBase {

    /** A resolved alignment goal in camera space (meters). */
    public static class TrackingGoal {
        /** Desired camera-space X of the tag (+ = tag to the robot's right). */
        public final double lateral;
        /** Desired camera-space Z distance to the tag. */
        public final double distance;

        public TrackingGoal(double lateral, double distance) {
            this.lateral = lateral;
            this.distance = distance;
        }
    }

    private final Swerve swerve;

    private boolean trackingEnabled = false;
    private boolean positionTrackingEnabled = true;

    // Best target cache, refreshed once per periodic()
    private Optional<AprilTagTarget> cachedBestTarget = Optional.empty();

    // Precomputed "limelight-<name>" NT table names (avoids per-loop string
    // concatenation in the hot paths)
    private final String[] limelightTableNames;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        limelightTableNames = new String[VisionConstants.LIMELIGHT_NAMES.length];
        for (int i = 0; i < VisionConstants.LIMELIGHT_NAMES.length; i++) {
            limelightTableNames[i] = "limelight-" + VisionConstants.LIMELIGHT_NAMES[i];
        }

        for (String tableName : limelightTableNames) {
            // Set all Limelights to the AprilTag pipeline
            LimelightHelpers.setPipelineIndex(tableName, VisionConstants.APRILTAG_PIPELINE);
            // Turn off Limelight LEDs during initialization
            LimelightHelpers.setLEDMode_ForceOff(tableName);
        }
    }

    /** Whether the Limelight at the given LIMELIGHT_NAMES index sees a target. */
    public boolean hasTarget(int index) {
        return LimelightHelpers.getTV(limelightTableNames[index]);
    }

    // Toggles AprilTag tracking and controls Limelight LEDs.
    public void toggleTracking(boolean enabled) {
        if (enabled != trackingEnabled) {
            trackingEnabled = enabled;
            for (String tableName : limelightTableNames) {
                if (trackingEnabled) {
                    LimelightHelpers.setLEDMode_ForceOn(tableName);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff(tableName);
                }
            }
        }
    }

    // Returns whether AprilTag tracking is enabled.
    // NOTE: intentionally unused - kept as the getter paired with
    // toggleTracking() for dashboards/tests (Swerve tracks its own copy of
    // this state for the button toggle).
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    private static boolean isReefTag(int tagId) {
        for (int tag : VisionConstants.REEF_TAGS) {
            if (tag == tagId) {
                return true;
            }
        }
        return false;
    }

    private static boolean isCoralStationTag(int tagId) {
        for (int tag : VisionConstants.CORAL_STATION_TAGS) {
            if (tag == tagId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Resolves the alignment goal for a tag from the tag's class alone.
     * Returns empty for tags that are intentionally not tracked (barge and
     * processor) - the tracker treats those like no target at all.
     *
     *  - Reef tags: bumpers flush with the reef base, centered on the tag
     *    (the KitBot ejects into the L1 trough from there).
     *  - Coral station tags: bumpers flush with the station wall, centered.
     */
    public Optional<TrackingGoal> getTrackingGoal(int tagId) {
        if (isReefTag(tagId)) {
            return Optional.of(new TrackingGoal(0.0, VisionConstants.REEF_FLUSH_DISTANCE));
        }
        if (isCoralStationTag(tagId)) {
            return Optional.of(new TrackingGoal(0.0, VisionConstants.STATION_FLUSH_DISTANCE));
        }
        // Barge (4, 5, 14, 15) and processor (3, 16): intentionally blank
        return Optional.empty();
    }

    /**
     * The best (closest) AprilTag target seen this loop (cached; refreshed
     * in periodic).
     */
    public Optional<AprilTagTarget> getBestTarget() {
        return cachedBestTarget;
    }

    // Queries every Limelight and picks the CLOSEST visible TRACKABLE tag
    // (smallest ground distance in camera space). Tags with no tracking
    // goal (barge/processor) are skipped entirely.
    private Optional<AprilTagTarget> findBestTarget() {
        AprilTagTarget bestTarget = null;
        double bestDistance = Double.MAX_VALUE;

        for (int i = 0; i < limelightTableNames.length; i++) {
            String name = VisionConstants.LIMELIGHT_NAMES[i];
            String limelightName = limelightTableNames[i];
            if (!LimelightHelpers.getTV(limelightName)) {
                continue;
            }

            int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
            if (getTrackingGoal(tagId).isEmpty()) {
                continue; // Intentionally untracked tag class
            }

            // Limelight camera space: X = right, Y = down, Z = forward.
            // Ground distance ignores the vertical (Y) axis.
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
            double distance = Math.hypot(targetPose.getX(), targetPose.getZ());
            if (distance < bestDistance) {
                bestTarget = new AprilTagTarget(
                    tagId,
                    LimelightHelpers.getTX(limelightName),
                    LimelightHelpers.getTY(limelightName),
                    targetPose.getX(),
                    targetPose.getY(),
                    targetPose.getZ(),
                    name,
                    VisionConstants.LIMELIGHT_FACING_SIGNS[i]
                );
                bestDistance = distance;
            }
        }

        return Optional.ofNullable(bestTarget);
    }

    /**
     * Enables or disables MegaTag2 pose fusion. Useful during testing to
     * compare pure wheel odometry against vision-corrected odometry.
     *
     * NOTE: intentionally unused (with its getter below) - fusion defaults
     * to on; call or temporarily bind this in test sessions only.
     */
    public void enablePositionTracking(boolean enabled) {
        positionTrackingEnabled = enabled;
    }

    public boolean isPositionTrackingEnabled() {
        return positionTrackingEnabled;
    }

    /**
     * An AprilTag target detected by a Limelight, in camera space:
     * poseX = right (m), poseY = down (m), poseZ = forward (m).
     * Driving math uses X (lateral) and Z (distance); Y is display-only.
     */
    public static class AprilTagTarget {
        public final int id;
        public final double tx; // Horizontal angle to target (degrees, + = right)
        public final double ty; // Vertical angle to target (degrees)
        public final double poseX;
        public final double poseY;
        public final double poseZ;
        public final String limelightName;
        /** +1 if the reporting camera faces the robot's front, -1 if the rear. */
        public final double facingSign;

        public AprilTagTarget(int id, double tx, double ty,
                double poseX, double poseY, double poseZ, String limelightName,
                double facingSign) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.poseX = poseX;
            this.poseY = poseY;
            this.poseZ = poseZ;
            this.limelightName = limelightName;
            this.facingSign = facingSign;
        }

        /** Ground-plane distance from the camera to the tag, in meters. */
        public double groundDistance() {
            return Math.hypot(poseX, poseZ);
        }
    }

    /**
     * Fuses vision pose estimates from every Limelight into the drivetrain
     * odometry. Timestamp conversion to the Phoenix timebase happens inside
     * Swerve.addVisionMeasurement().
     *
     * Two modes:
     *  - DISABLED (pre-match / between periods): MegaTag1, whose solve
     *    includes an absolute HEADING from tag geometry alone. Its rotation
     *    is fused so the pose heading converges to field-correct while the
     *    robot sits still - without this, the heading MegaTag2 depends on
     *    would start at whatever the gyro booted to and every fused pose
     *    would be wrong until the first manual pose/heading reset.
     *  - ENABLED: MegaTag2, which takes our (now-seeded) heading and returns
     *    a far more stable translation than single-tag solves. Its heading
     *    is our own gyro echoed back, so it gets effectively zero weight.
     */
    private void updateRobotPosition() {
        boolean seedingHeading = DriverStation.isDisabled();
        double headingDegrees = swerve.getState().Pose.getRotation().getDegrees();

        for (String tableName : limelightTableNames) {
            // NoFlush variant: one NT flush after the loop instead of a full
            // network flush per camera per loop
            LimelightHelpers.SetRobotOrientation_NoFlush(tableName, headingDegrees, 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate estimate = seedingHeading
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tableName);

            if (estimate == null || estimate.tagCount == 0) {
                continue;
            }

            // Confidence scales with tag count and closeness
            double xyStdDev = 0.3
                + 0.4 * (estimate.avgTagDist * estimate.avgTagDist) / Math.max(1, estimate.tagCount);
            // MegaTag1 heading is trusted (loosely; tighter with 2+ tags)
            // while disabled and stationary; MegaTag2 heading never is.
            double rotStdDev = seedingHeading
                ? (estimate.tagCount >= 2 ? 0.3 : 0.9)
                : 9999999;

            swerve.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));
        }

        LimelightHelpers.Flush();
    }

    @Override
    public void periodic() {
        // Refresh the best-target cache once per loop; all readers use this.
        cachedBestTarget = findBestTarget();

        if (positionTrackingEnabled) {
            updateRobotPosition();
        }
    }
}
