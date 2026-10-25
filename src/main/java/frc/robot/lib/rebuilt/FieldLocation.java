package frc.robot.lib.rebuilt;

import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

/** Alliance-aware Rebuilt pathfinding destinations, derived from the official 2026 tag layout. */
public enum FieldLocation {
    LEFT_TRENCH(RebuiltField.leftTrench(), true, false),
    RIGHT_TRENCH(RebuiltField.rightTrench(), true, false),
    DEPOT(RebuiltField.depot(), true, false),
    HUB_NEAR(RebuiltField.hubStagingPose(26), false, true),
    HUB_FAR(RebuiltField.hubStagingPose(20), false, true),
    HUB_LEFT(RebuiltField.hubStagingPose(21), false, true),
    HUB_RIGHT(RebuiltField.hubStagingPose(18), false, true),
    ALLIANCE_LEFT_CORNER(RebuiltField.leftCorner(), false, false),
    ALLIANCE_RIGHT_CORNER(RebuiltField.rightCorner(), false, false);

    private final Pose2d bluePose;
    private final boolean collectionLocation;
    private final boolean hubStagingLocation;

    FieldLocation(Pose2d bluePose, boolean collectionLocation, boolean hubStagingLocation) {
        this.bluePose = bluePose;
        this.collectionLocation = collectionLocation;
        this.hubStagingLocation = hubStagingLocation;
    }

    public Pose2d getPose(boolean isRedAlliance) {
        return isRedAlliance ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    public boolean isCollectionLocation() {
        return collectionLocation;
    }

    public boolean isHubStagingLocation() {
        return hubStagingLocation;
    }

    /** Rebuilt geometry used only for navigation targets; keep mechanism code independent of it. */
    private static final class RebuiltField {
        private static final AprilTagFieldLayout LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        private static final Translation2d BLUE_HUB_CENTER =
                new Translation2d(Units.inchesToMeters(158.6), LAYOUT.getFieldWidth() / 2.0);
        private static final double HUB_STAGING_RADIUS_METERS = 2.4;
        // Keep the entire bumper inside the field at every corner target orientation.
        private static final double CORNER_STANDOFF_METERS =
                Math.hypot(
                                Units.inchesToMeters(DriveConstants.kBumperLengthInches / 2.0),
                                Units.inchesToMeters(DriveConstants.kBumperWidthInches / 2.0))
                        + Units.inchesToMeters(2.0);

        static AprilTagFieldLayout layout() {
            return LAYOUT;
        }

        static Pose2d tagPose(int id) {
            return LAYOUT.getTagPose(id)
                    .orElseThrow(() -> new IllegalStateException("Missing Rebuilt tag " + id))
                    .toPose2d();
        }

        /** Returns a collision-safe shooting pose outside the 47 in square Hub perimeter. */
        static Pose2d hubStagingPose(int tagId) {
            Translation2d outward = tagPose(tagId).getTranslation().minus(BLUE_HUB_CENTER);
            if (outward.getNorm() < 1e-6) {
                throw new IllegalStateException("Rebuilt Hub tag is at the Hub center: " + tagId);
            }

            Translation2d position =
                    BLUE_HUB_CENTER.plus(
                            outward.times(HUB_STAGING_RADIUS_METERS / outward.getNorm()));
            return new Pose2d(position, BLUE_HUB_CENTER.minus(position).getAngle());
        }

        static Pose2d depot() {
            return new Pose2d(
                    Units.inchesToMeters(27.0),
                    LAYOUT.getFieldWidth() / 2.0 + Units.inchesToMeters(75.93),
                    Rotation2d.kPi);
        }

        static Pose2d leftCorner() {
            return new Pose2d(
                    CORNER_STANDOFF_METERS,
                    LAYOUT.getFieldWidth() - CORNER_STANDOFF_METERS,
                    Rotation2d.kZero);
        }

        static Pose2d rightCorner() {
            return new Pose2d(CORNER_STANDOFF_METERS, CORNER_STANDOFF_METERS, Rotation2d.kZero);
        }

        static Pose2d leftTrench() {
            return new Pose2d(
                    tagPose(26).getX() + Units.inchesToMeters(23.5),
                    LAYOUT.getFieldWidth() - Units.inchesToMeters(25.17),
                    new Rotation2d());
        }

        static Pose2d rightTrench() {
            return new Pose2d(
                    tagPose(26).getX() + Units.inchesToMeters(23.5),
                    Units.inchesToMeters(25.17),
                    new Rotation2d());
        }
    }
}
