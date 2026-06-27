package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the conveyor-style indexer. */
public final class IndexerConstants {

    private IndexerConstants() {}

    private static final int CONVEYOR_TUNABLE_ID = "IndexerConstantsConveyor".hashCode();

    public static final int CONVEYOR_MOTOR_ID = 32;
    public static final CANBus CAN_BUS = new CANBus("swerve");

    public static final double CONVEYOR_GEAR_RATIO = 9.58333333;
    public static final double CONVEYOR_STATOR_CURRENT_LIMIT = 40.0;

    public static final LoggedTunableNumber CONVEYOR_KS =
            new LoggedTunableNumber("Indexer/Conveyor/kS", 4.0);
    public static final LoggedTunableNumber CONVEYOR_KV =
            new LoggedTunableNumber("Indexer/Conveyor/kV", 0.12);
    public static final LoggedTunableNumber CONVEYOR_KA =
            new LoggedTunableNumber("Indexer/Conveyor/kA", 0.0);
    public static final LoggedTunableNumber CONVEYOR_KP =
            new LoggedTunableNumber("Indexer/Conveyor/kP", 10.0);
    public static final LoggedTunableNumber CONVEYOR_KI =
            new LoggedTunableNumber("Indexer/Conveyor/kI", 0.0);
    public static final LoggedTunableNumber CONVEYOR_KD =
            new LoggedTunableNumber("Indexer/Conveyor/kD", 0.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;
    public static final double CONVEYOR_MOI = 0.002;

    public static boolean hasAnyConveyorPIDChanged() {
        return CONVEYOR_KS.hasChanged(CONVEYOR_TUNABLE_ID)
                || CONVEYOR_KV.hasChanged(CONVEYOR_TUNABLE_ID)
                || CONVEYOR_KA.hasChanged(CONVEYOR_TUNABLE_ID)
                || CONVEYOR_KP.hasChanged(CONVEYOR_TUNABLE_ID)
                || CONVEYOR_KI.hasChanged(CONVEYOR_TUNABLE_ID)
                || CONVEYOR_KD.hasChanged(CONVEYOR_TUNABLE_ID);
    }
}
