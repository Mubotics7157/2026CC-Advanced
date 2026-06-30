package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the shooter feeder wheel. */
public final class FeederConstants {

    private FeederConstants() {}

    private static final int FEEDER_TUNABLE_ID = "FeederConstantsFeeder".hashCode();

    public static final int FEEDER_MOTOR_ID = 27;
    public static final CANBus CAN_BUS = new CANBus("swerve");

    public static final double FEEDER_GEAR_RATIO = 1.0;
    public static final double FEEDER_STATOR_CURRENT_LIMIT = 120.0;

    public static final LoggedTunableNumber FEEDER_KS =
            new LoggedTunableNumber("Feeder/Feeder/kS", 0.0);
    public static final LoggedTunableNumber FEEDER_KV =
            new LoggedTunableNumber("Feeder/Feeder/kV", 0.12);
    public static final LoggedTunableNumber FEEDER_KA =
            new LoggedTunableNumber("Feeder/Feeder/kA", 0.0);
    public static final LoggedTunableNumber FEEDER_KP =
            new LoggedTunableNumber("Feeder/Feeder/kP", 1.0);
    public static final LoggedTunableNumber FEEDER_KI =
            new LoggedTunableNumber("Feeder/Feeder/kI", 0.0);
    public static final LoggedTunableNumber FEEDER_KD =
            new LoggedTunableNumber("Feeder/Feeder/kD", 0.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;
    public static final double FEEDER_MOI = 0.001;

    public static boolean hasAnyFeederPIDChanged() {
        return FEEDER_KS.hasChanged(FEEDER_TUNABLE_ID)
                || FEEDER_KV.hasChanged(FEEDER_TUNABLE_ID)
                || FEEDER_KA.hasChanged(FEEDER_TUNABLE_ID)
                || FEEDER_KP.hasChanged(FEEDER_TUNABLE_ID)
                || FEEDER_KI.hasChanged(FEEDER_TUNABLE_ID)
                || FEEDER_KD.hasChanged(FEEDER_TUNABLE_ID);
    }
}
