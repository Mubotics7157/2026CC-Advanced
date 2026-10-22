package frc.robot.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Field-wide constants sourced from the official 2026 Rebuilt AprilTag layout. */
public final class FieldConstants {
    private FieldConstants() {}

    public static final AprilTagFieldLayout kAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = kAprilTagLayout.getFieldLength();
    public static final double fieldWidth = kAprilTagLayout.getFieldWidth();
    public static final double kFieldLengthMeters = fieldLength;
    public static final double kFieldWidthMeters = fieldWidth;

    /** Keep autonomous possession inference away from the center-line transition zone. */
    public static final double kMidlineBuffer = 1.0;
}
