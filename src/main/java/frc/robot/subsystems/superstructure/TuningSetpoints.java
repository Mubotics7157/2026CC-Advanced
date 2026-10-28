package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Dashboard targets applied to all mechanisms while {@code Constants.kTuningMode} is enabled. */
public final class TuningSetpoints {
    private TuningSetpoints() {}

    public static final LoggedTunableNumber INTAKE_ROLLER_RAD_PER_SEC =
            new LoggedTunableNumber("Tuning/Intake/RollerRadPerSec", 0.0);
    public static final LoggedTunableNumber INTAKE_ARM_RAD =
            new LoggedTunableNumber("Tuning/Intake/ArmRad", 0.0);
    public static final LoggedTunableNumber INDEXER_CONVEYOR_RAD_PER_SEC =
            new LoggedTunableNumber("Tuning/Indexer/ConveyorRadPerSec", 0.0);
    public static final LoggedTunableNumber INDEXER_FEEDER_RAD_PER_SEC =
            new LoggedTunableNumber("Tuning/Indexer/FeederRadPerSec", 0.0);
    public static final LoggedTunableNumber SHOOTER_RPM =
            new LoggedTunableNumber("Tuning/Shooter/FlywheelRPM", 0.0);
    public static final LoggedTunableNumber SHOOTER_HOOD_DEG =
            new LoggedTunableNumber("Tuning/Shooter/HoodAngleDeg", 20.0);

    public static double shooterVelocityRadPerSec() {
        return Units.rotationsPerMinuteToRadiansPerSecond(SHOOTER_RPM.get());
    }

    public static double hoodAngleRad() {
        return Units.degreesToRadians(SHOOTER_HOOD_DEG.get());
    }
}
