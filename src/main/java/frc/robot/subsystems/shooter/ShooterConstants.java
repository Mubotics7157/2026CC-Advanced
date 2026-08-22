package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the three-wheel shooter. */
public final class ShooterConstants {

    private ShooterConstants() {}

    private static final int SHOOTER_TUNABLE_ID = "ShooterConstantsShooter".hashCode();

    public static final int LEFT_MOTOR_ID = 24;
    public static final int MIDDLE_MOTOR_ID = 16;
    public static final int RIGHT_MOTOR_ID = 20;
    public static final CANBus CAN_BUS = new CANBus("swerve");

    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 120.0;

    public static final double VELOCITY_TOLERANCE_RAD_PER_SEC =
            Units.rotationsPerMinuteToRadiansPerSecond(100.0);
    public static final double HOOD_MIN_ANGLE_RAD = Units.degreesToRadians(20.0);
    public static final double HOOD_MAX_ANGLE_RAD = Units.degreesToRadians(65.0);
    public static final double HOOD_POSITION_TOLERANCE_RAD = Units.degreesToRadians(1.5);

    /*
     * Set these to the real PCM/PH and sensor channels. With HOOD_POSITION_SENSOR_CHANNEL < 0,
     * hardware will command no hood pneumatics and will log the requested angle for bring-up.
     */
    public static final int PNEUMATICS_MODULE_ID = 1;
    public static final int HOOD_PRIMARY_FORWARD_CHANNEL = -1;
    public static final int HOOD_PRIMARY_REVERSE_CHANNEL = -1;
    public static final int HOOD_SECONDARY_FORWARD_CHANNEL = -1;
    public static final int HOOD_SECONDARY_REVERSE_CHANNEL = -1;
    public static final int HOOD_POSITION_SENSOR_CHANNEL = -1;
    public static final double HOOD_SENSOR_MIN_VOLTS = 0.5;
    public static final double HOOD_SENSOR_MAX_VOLTS = 4.5;

    public static final LoggedTunableNumber SHOOTER_KS =
            new LoggedTunableNumber("Shooter/Shooter/kS", 0.0);
    public static final LoggedTunableNumber SHOOTER_KV =
            new LoggedTunableNumber("Shooter/Shooter/kV", 0.12);
    public static final LoggedTunableNumber SHOOTER_KA =
            new LoggedTunableNumber("Shooter/Shooter/kA", 0.0);
    public static final LoggedTunableNumber SHOOTER_KP =
            new LoggedTunableNumber("Shooter/Shooter/kP", 0.5);
    public static final LoggedTunableNumber SHOOTER_KI =
            new LoggedTunableNumber("Shooter/Shooter/kI", 2.0);
    public static final LoggedTunableNumber SHOOTER_KD =
            new LoggedTunableNumber("Shooter/Shooter/kD", 0.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;
    public static final double SHOOTER_MOI = 0.003;

    public static boolean hasAnyShooterPIDChanged() {
        return SHOOTER_KS.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KV.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KA.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KP.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KI.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KD.hasChanged(SHOOTER_TUNABLE_ID);
    }
}
