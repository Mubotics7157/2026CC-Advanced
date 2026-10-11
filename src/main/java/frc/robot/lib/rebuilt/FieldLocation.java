package frc.robot.lib.rebuilt;

import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Alliance-aware Rebuilt pathfinding destinations, derived from the official 2026 tag layout. */
public enum FieldLocation {
    LEFT_TRENCH(RebuiltField.leftTrench()),
    RIGHT_TRENCH(RebuiltField.rightTrench()),
    DEPOT(RebuiltField.depot()),
    HUB_NEAR(RebuiltField.tagPose(26)),
    HUB_FAR(RebuiltField.tagPose(20)),
    HUB_LEFT(RebuiltField.tagPose(21)),
    HUB_RIGHT(RebuiltField.tagPose(18)),
    ALLIANCE_LEFT_CORNER(new Pose2d(0.0, RebuiltField.layout().getFieldWidth(), new Rotation2d())),
    ALLIANCE_RIGHT_CORNER(new Pose2d());

    private final Pose2d bluePose;

    FieldLocation(Pose2d bluePose) {
        this.bluePose = bluePose;
    }

    public Pose2d getPose(boolean isRedAlliance) {
        return isRedAlliance ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    /** Rebuilt geometry used only for navigation targets; keep mechanism code independent of it. */
    private static final class RebuiltField {
        private static final AprilTagFieldLayout LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        static AprilTagFieldLayout layout() {
            return LAYOUT;
        }

        static Pose2d tagPose(int id) {
            return LAYOUT.getTagPose(id)
                    .orElseThrow(() -> new IllegalStateException("Missing Rebuilt tag " + id))
                    .toPose2d();
        }

        static Pose2d depot() {
            return new Pose2d(
                    Units.inchesToMeters(27.0),
                    LAYOUT.getFieldWidth() / 2.0 + Units.inchesToMeters(75.93),
                    Rotation2d.kPi);
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
