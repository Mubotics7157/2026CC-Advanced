package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;

public final class SuperstructureConstants {

    private SuperstructureConstants() {}

    /** Call this method at robot startup to ensure constants are initialized. */
    public static void init() {}

    public static final double INTAKE_ROLLER_VELOCITY_RAD_PER_SEC = 500.0;
    public static final double INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC = 80.0;
    public static final double FEEDER_VELOCITY_RAD_PER_SEC =
            Units.rotationsPerMinuteToRadiansPerSecond(5000.0);
    public static final double SHOOTER_VELOCITY_RAD_PER_SEC =
            Units.rotationsPerMinuteToRadiansPerSecond(2600.0);
    public static final double SHOOTER_HOOD_ANGLE_RAD = Units.degreesToRadians(35.0);
    public static final double SHOOTER_FEED_DELAY_SECONDS = 0.25;

    /** Full back-and-forth forebar cycle used to agitate balls while feeding the shooter. */
    public static final double SHOOTING_AGITATION_PERIOD_SECONDS = 0.25;
}
