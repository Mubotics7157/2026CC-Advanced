package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.FieldConstants;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.Util;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Distance-based shot maps and target geometry for the non-turret drum shooter. */
public final class ShooterSetpoints {

    private ShooterSetpoints() {}

    public static final double MIN_SHOOTING_DISTANCE_METERS = 1.5;
    public static final double MAX_SHOOTING_DISTANCE_METERS = 6.0;

    /*
     * Default target is the center of the alliance scoring target. These are intentionally easy
     * to retune once the real 2026 field target is finalized/measured.
     */
    public static final Translation2d BLUE_TARGET_TRANSLATION =
            new Translation2d(Units.inchesToMeters(66.0), FieldConstants.fieldWidth / 2.0);
    public static final Translation2d RED_TARGET_TRANSLATION =
            Util.flipRedBlue(BLUE_TARGET_TRANSLATION);

    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(72.0);
    public static final double SHOOTER_EXIT_HEIGHT_METERS = Units.inchesToMeters(24.0);

    /** Robot-relative position of the drum exit, assuming the shooter fires along robot +X. */
    public static final Translation2d ROBOT_TO_SHOOTER_EXIT =
            new Translation2d(Units.inchesToMeters(12.0), 0.0);

    /** Add this if the drum exit is not aligned with robot +X. */
    public static final double ROBOT_SHOOTING_YAW_OFFSET_RAD = 0.0;

    public static final double PRELAUNCH_LOOKAHEAD_SECONDS = 0.05;
    public static final int LAUNCH_LOOKAHEAD_ITERATIONS = 4;
    public static final double HEADING_TOLERANCE_RAD = Units.degreesToRadians(2.0);
    public static final double OMEGA_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(12.0);

    /** Distance (meters) to drum wheel velocity (RPM). Replace these with characterized data. */
    public static final InterpolatingTreeMap SHOOTER_VELOCITY_RPM_MAP =
            new InterpolatingTreeMap()
                    .put(1.5, 2200.0)
                    .put(2.0, 2400.0)
                    .put(3.0, 2700.0)
                    .put(4.0, 3100.0)
                    .put(5.0, 3500.0)
                    .put(6.0, 3900.0);

    /** Distance (meters) to requested hood angle (radians). */
    public static final InterpolatingTreeMap HOOD_ANGLE_MAP =
            new InterpolatingTreeMap()
                    .put(1.5, Units.degreesToRadians(22.0))
                    .put(2.0, Units.degreesToRadians(26.0))
                    .put(3.0, Units.degreesToRadians(34.0))
                    .put(4.0, Units.degreesToRadians(43.0))
                    .put(5.0, Units.degreesToRadians(53.0))
                    .put(6.0, Units.degreesToRadians(62.0));

    /** Distance (meters) to flight time (seconds). Used for shoot-on-the-move lookahead. */
    public static final InterpolatingTreeMap TIME_OF_FLIGHT_SECONDS_MAP =
            new InterpolatingTreeMap()
                    .put(1.5, 0.45)
                    .put(2.0, 0.55)
                    .put(3.0, 0.70)
                    .put(4.0, 0.85)
                    .put(5.0, 1.00)
                    .put(6.0, 1.15);

    public static final LoggedTunableNumber SHOOTER_RPM_OFFSET =
            new LoggedTunableNumber("Shooter/ShotMap/RpmOffset", 0.0);
    public static final LoggedTunableNumber HOOD_ANGLE_OFFSET_DEG =
            new LoggedTunableNumber("Shooter/ShotMap/HoodAngleOffsetDeg", 0.0);

    public static Translation2d getTargetTranslation(boolean isRedAlliance) {
        return isRedAlliance ? RED_TARGET_TRANSLATION : BLUE_TARGET_TRANSLATION;
    }

    public static boolean isInRange(double distanceMeters) {
        return distanceMeters >= MIN_SHOOTING_DISTANCE_METERS
                && distanceMeters <= MAX_SHOOTING_DISTANCE_METERS;
    }

    public static double getVelocityRadPerSec(double distanceMeters) {
        double rpm = SHOOTER_VELOCITY_RPM_MAP.get(distanceMeters) + SHOOTER_RPM_OFFSET.get();
        Logger.recordOutput("Shooter/ShotMap/InterpolatedRPM", rpm);
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    public static double getHoodAngleRad(double distanceMeters) {
        double angleRad =
                HOOD_ANGLE_MAP.get(distanceMeters)
                        + Units.degreesToRadians(HOOD_ANGLE_OFFSET_DEG.get());
        Logger.recordOutput("Shooter/ShotMap/InterpolatedHoodAngleRad", angleRad);
        Logger.recordOutput("Shooter/ShotMap/InterpolatedHoodAngleDeg", Math.toDegrees(angleRad));
        return angleRad;
    }

    public static double getTimeOfFlightSeconds(double distanceMeters) {
        return TIME_OF_FLIGHT_SECONDS_MAP.get(distanceMeters);
    }
}
