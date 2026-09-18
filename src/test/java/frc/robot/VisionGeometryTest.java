package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraPose;
import frc.robot.Constants.VisionConstants.TagClass;
import frc.robot.subsystems.Vision;

/**
 * Pins the camera-space -> robot-frame conversion the alignment tracker
 * relies on, the field-layout headings it squares up to (against the
 * rotations the authored paths end at), the goals it drives to, the tag
 * classes, and the dashboard's unfiltered "Best Tag" / "Visible Tags"
 * selection. Pure geometry - no HAL, and no camera: this robot has none
 * mounted, so the camera poses here are examples local to the test.
 */
class VisionGeometryTest {

    private static final double EPS = 1e-3;

    @Test
    void unpitchedForwardCameraAtCenterIsIdentity() {
        CameraPose cam = new CameraPose(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, true);
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.5, 0.0, 2.0);
        assertEquals(2.0, t.getX(), EPS, "camera Z (forward) becomes robot X");
        assertEquals(-0.5, t.getY(), EPS, "camera X (right) becomes robot -Y");
        assertFalse(cam.facesRear());
    }

    @Test
    void offsetYawAndPitchArePlacedOnTheRobot() {
        // 0.28 m forward, 0.29 m to the robot's LEFT (Limelight side = -0.29),
        // pitched 20 deg down, yawed 30 deg to the right
        CameraPose cam = new CameraPose(0.28, -0.29, 0.40, 0.0, -20.0, -30.0, true);
        // A tag on the optical axis 2 m away lies 2 cos(20) along the camera heading
        double along = 2.0 * Math.cos(Math.toRadians(20.0));
        Translation2d expected = new Translation2d(0.28, 0.29)
            .plus(new Translation2d(along, 0.0).rotateBy(Rotation2d.fromDegrees(-30.0)));
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.0, 0.0, 2.0);
        assertEquals(expected.getX(), t.getX(), EPS);
        assertEquals(expected.getY(), t.getY(), EPS);
        assertTrue(t.getY() < 0.0, "an on-axis tag is to the robot's RIGHT of center for a right-yawed camera");
    }

    @Test
    void rearCameraPutsTheTagBehindTheRobot() {
        // 0.35 m behind center, pitched 50 deg up, facing straight back
        CameraPose cam = new CameraPose(-0.35, 0.0, 0.75, 0.0, 50.0, 180.0, true);
        assertTrue(cam.facesRear());
        Translation2d t = Vision.tagPositionInRobotFrame(cam, 0.0, 0.0, 1.0);
        assertEquals(-(0.35 + Math.cos(Math.toRadians(50.0))), t.getX(), EPS);
        assertEquals(0.0, t.getY(), EPS);
        // Image-down (camera +Y) points along-the-heading-and-down for an
        // up-tilted camera (the bottom of the image looks toward the
        // horizon), so a point lower in the image at the same depth is
        // further out: behind a rear camera, further behind the robot
        Translation2d lower = Vision.tagPositionInRobotFrame(cam, 0.0, 0.5, 1.0);
        assertTrue(lower.getX() < t.getX(), "a point lower in the image is further behind the up-tilted rear camera");
        // Camera right is the robot's LEFT for a rear-facing camera
        Translation2d right = Vision.tagPositionInRobotFrame(cam, 0.3, 0.0, 1.0);
        assertEquals(0.3, right.getY(), EPS);
    }

    /**
     * The approach sides are this robot's mechanism layout: the KitBot
     * roller ejects out of the REAR, the coral is loaded at the FRONT. The
     * authored autos are the record of that - every reef path ends backed
     * up to its reef face and every station path ends facing the station -
     * so the headings the aligner squares up to must equal the rotations
     * those paths end at.
     */
    @Test
    void squareHeadingsMatchTheAuthoredPathEndRotations() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);
        boolean reef = Vision.approachesRear(TagClass.REEF);
        boolean station = Vision.approachesRear(TagClass.CORAL_STATION);
        assertTrue(reef, "the reef is backed up to");
        assertFalse(station, "a coral station is approached nose-first");
        // "Left / Right Station to Front Reef" end at 180: backed up to tag 18, which faces -X
        assertEquals(180.0, Math.abs(Vision.squareHeading(layout, 18, reef).orElseThrow().getDegrees()), 0.5);
        // "Left Station to Left Reef" ends at 120 (tag 19), "Left Cage to Back Left Reef" at 60 (tag 20)
        assertEquals(120.0, Vision.squareHeading(layout, 19, reef).orElseThrow().getDegrees(), 0.5);
        assertEquals(60.0, Vision.squareHeading(layout, 20, reef).orElseThrow().getDegrees(), 0.5);
        // "Center Drop" ends at 0: backed up to tag 21, which faces +X
        assertEquals(0.0, Vision.squareHeading(layout, 21, reef).orElseThrow().getDegrees(), 0.5);
        // "Right Station to Right Reef" ends at -120 (tag 17)
        assertEquals(-120.0, Vision.squareHeading(layout, 17, reef).orElseThrow().getDegrees(), 0.5);
        // The station paths end at 126 (left station, tag 13) and -126 (right station, tag 12), facing the wall
        assertEquals(126.0, Vision.squareHeading(layout, 13, station).orElseThrow().getDegrees(), 0.5);
        assertEquals(-126.0, Vision.squareHeading(layout, 12, station).orElseThrow().getDegrees(), 0.5);
        // The opposite approach is the opposite heading
        assertEquals(0.0, Vision.squareHeading(layout, 18, false).orElseThrow().getDegrees(), 0.5);
        assertEquals(Optional.empty(), Vision.squareHeading(layout, 99, reef));
        assertEquals(Optional.empty(), Vision.squareHeading(null, 18, reef), "no layout, no heading");
        // The layout is the 2025 field the flip constants describe
        assertEquals(VisionConstants.FIELD_LENGTH_METERS, layout.getFieldLength(), 1e-3);
        assertEquals(VisionConstants.FIELD_WIDTH_METERS, layout.getFieldWidth(), 1e-3);
    }

    @Test
    void goalsAreFlushAndCenteredAtTheEndThatClassIsApproachedFrom() {
        Vision.TrackingGoal reef = Vision.trackingGoal(TagClass.REEF, 0.47, 0.50).orElseThrow();
        assertEquals(-0.47, reef.forward, EPS, "reef: the tag ends up BEHIND the robot center (rear bumper flush)");
        assertEquals(0.0, reef.left, EPS, "reef: centered on the face - the L1 trough has no left / right branch");
        Vision.TrackingGoal station = Vision.trackingGoal(TagClass.CORAL_STATION, 0.47, 0.50).orElseThrow();
        assertEquals(0.50, station.forward, EPS, "station: the tag ends up AHEAD of the robot center (front bumper flush)");
        assertEquals(0.0, station.left, EPS);
        assertTrue(Vision.trackingGoal(TagClass.NONE, 0.47, 0.50).isEmpty(), "barge / processor / unknown: no goal");
    }

    @Test
    void fieldAndRobotFrameRoundTripThroughTheRobotPose() {
        // Robot at (1, 2) facing +Y (90 deg): a tag 1 m straight ahead is at field (1, 3)
        Pose2d robot = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));
        Translation2d ahead = new Translation2d(1.0, 0.0);
        Translation2d onField = Vision.robotFrameToField(robot, ahead);
        assertEquals(1.0, onField.getX(), EPS);
        assertEquals(3.0, onField.getY(), EPS);
        Translation2d back = Vision.fieldToRobotFrame(robot, onField);
        assertEquals(1.0, back.getX(), EPS);
        assertEquals(0.0, back.getY(), EPS);
        // After the robot drives 0.4 m forward, the remembered tag is 0.6 m ahead
        Pose2d moved = new Pose2d(1.0, 2.4, Rotation2d.fromDegrees(90.0));
        assertEquals(0.6, Vision.fieldToRobotFrame(moved, onField).getX(), EPS);
        // A tag to the robot's LEFT (+Y) when facing +Y is at smaller field X
        Translation2d leftOnField = Vision.robotFrameToField(robot, new Translation2d(0.0, 0.5));
        assertEquals(0.5, leftOnField.getX(), EPS);
        assertEquals(2.0, leftOnField.getY(), EPS);
    }

    @Test
    void tagClassesFollowTheManual() {
        assertEquals(TagClass.REEF, Vision.classOf(21));
        assertEquals(TagClass.REEF, Vision.classOf(6));
        assertEquals(TagClass.CORAL_STATION, Vision.classOf(1));
        assertEquals(TagClass.CORAL_STATION, Vision.classOf(13));
        assertEquals(TagClass.NONE, Vision.classOf(3), "processor: nothing on this robot scores there");
        assertEquals(TagClass.NONE, Vision.classOf(14), "barge: nothing on this robot scores there");
        assertEquals(TagClass.NONE, Vision.classOf(99), "not a 2025 tag");
    }

    /** One "rawfiducials" entry: id, txnc, tync, ta, distToCamera, distToRobot, ambiguity. */
    private static double[] rawTag(int id, double area, double distance) {
        return new double[] {id, 0.0, 0.0, area, distance, distance, 0.1};
    }

    private static double[] concat(double[] a, double[] b) {
        double[] joined = new double[a.length + b.length];
        System.arraycopy(a, 0, joined, 0, a.length);
        System.arraycopy(b, 0, joined, a.length, b.length);
        return joined;
    }

    /**
     * The dashboard's "Best Tag" is the closest tag ANY camera reports - whatever its class, whichever
     * camera, measured pose or not - so it agrees with the camera streams. (The alignment cache filters
     * on all of those, so it cannot feed this readout: a stream would show a tag the widget ignores.)
     */
    @Test
    void bestTagIsTheClosestTagAnyCameraSees() {
        String[] cameras = {"front", "rear"};
        // front sees a barge tag (no alignment class at all) closest; rear sees two reef tags
        double[][] seen = {rawTag(14, 4.0, 1.5), concat(rawTag(18, 2.0, 2.0), rawTag(19, 0.5, 4.0))};
        Vision.SeenTag best = Vision.closestSeenTag(seen).orElseThrow();
        assertEquals(14, best.id);
        assertEquals(0, best.cameraIndex);
        assertEquals("front: 14 | rear: 18 19", Vision.seenTagsSummary(cameras, seen));

        // No 3D solve anywhere: the largest tag in the image stands in for the closest
        double[][] flat = {rawTag(12, 0.8, 0.0), rawTag(19, 2.5, 0.0)};
        assertEquals(19, Vision.closestSeenTag(flat).orElseThrow().id);

        // A camera that is not reporting, and a malformed array
        double[][] partial = {null, rawTag(20, 1.0, 1.2)};
        assertEquals("rear: 20", Vision.seenTagsSummary(cameras, partial));
        double[][] none = {new double[0], new double[] {1.0, 2.0}};
        assertTrue(Vision.closestSeenTag(none).isEmpty());
        assertEquals("", Vision.seenTagsSummary(cameras, none));

        // No camera configured at all (this robot today): nothing seen, nothing indexed
        assertTrue(Vision.closestSeenTag(new double[0][]).isEmpty());
        assertEquals("", Vision.seenTagsSummary(new String[0], new double[0][]));
    }
}
