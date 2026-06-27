package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the rack-and-pinion intake extension and its single roller. */
public final class IntakeConstants {

    private IntakeConstants() {}

    private static final int ROLLER_TUNABLE_ID = "IntakeConstantsRoller".hashCode();
    private static final int EXTENSION_TUNABLE_ID = "IntakeConstantsExtension".hashCode();

    public static final int ROLLER_MOTOR_ID = 22;
    public static final int EXTENSION_MOTOR_ID = 21;
    public static final CANBus CAN_BUS = new CANBus("swerve");

    public static final double ROLLER_GEAR_RATIO = 1.42857143;

    public static final double EXTENSION_MOTOR_TO_PINION_RATIO = 1.0;

    /**
     * Pinion rotations per meter of rack travel. Replace this with the measured rack/pinion
     * conversion once the final pinion pitch diameter is known.
     */
    public static final double EXTENSION_PINION_ROTATIONS_PER_METER = 25.0;

    public static final double EXTENSION_RETRACTED_POSITION_METERS = 0.0;
    public static final double EXTENSION_EXTENDED_POSITION_METERS = 0.45;
    public static final double EXTENSION_POSITION_TOLERANCE_METERS = 0.02;
    public static final double EXTENSION_DEADZONE_TOLERANCE_METERS = 0.01;

    public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0;
    public static final double EXTENSION_STATOR_CURRENT_LIMIT = 40.0;

    public static final LoggedTunableNumber ROLLER_KS =
            new LoggedTunableNumber("Intake/Roller/kS", 0.0);
    public static final LoggedTunableNumber ROLLER_KV =
            new LoggedTunableNumber("Intake/Roller/kV", 0.12);
    public static final LoggedTunableNumber ROLLER_KA =
            new LoggedTunableNumber("Intake/Roller/kA", 0.0);
    public static final LoggedTunableNumber ROLLER_KP =
            new LoggedTunableNumber("Intake/Roller/kP", 5.0);
    public static final LoggedTunableNumber ROLLER_KI =
            new LoggedTunableNumber("Intake/Roller/kI", 0.0);
    public static final LoggedTunableNumber ROLLER_KD =
            new LoggedTunableNumber("Intake/Roller/kD", 0.0);

    public static final LoggedTunableNumber EXTENSION_KS =
            new LoggedTunableNumber("Intake/Extension/kS", 0.0);
    public static final LoggedTunableNumber EXTENSION_KV =
            new LoggedTunableNumber("Intake/Extension/kV", 0.0);
    public static final LoggedTunableNumber EXTENSION_KA =
            new LoggedTunableNumber("Intake/Extension/kA", 0.0);
    public static final LoggedTunableNumber EXTENSION_KP =
            new LoggedTunableNumber("Intake/Extension/kP", 40.0);
    public static final LoggedTunableNumber EXTENSION_KI =
            new LoggedTunableNumber("Intake/Extension/kI", 0.0);
    public static final LoggedTunableNumber EXTENSION_KD =
            new LoggedTunableNumber("Intake/Extension/kD", 2.0);

    public static final LoggedTunableNumber EXTENSION_MOTION_MAGIC_EXPO_KV =
            new LoggedTunableNumber("Intake/Extension/MMExpo_kV", 0.12);
    public static final LoggedTunableNumber EXTENSION_MOTION_MAGIC_EXPO_KA =
            new LoggedTunableNumber("Intake/Extension/MMExpo_kA", 0.1);
    public static final LoggedTunableNumber EXTENSION_MOTION_MAGIC_CRUISE_VELOCITY =
            new LoggedTunableNumber("Intake/Extension/MMCruiseVelMetersPerSec", 1.0);
    public static final LoggedTunableNumber EXTENSION_MOTION_MAGIC_ACCELERATION =
            new LoggedTunableNumber("Intake/Extension/MMAccelMetersPerSecSq", 0.0);
    public static final LoggedTunableNumber EXTENSION_MOTION_MAGIC_JERK =
            new LoggedTunableNumber("Intake/Extension/MMJerkMetersPerSecCubed", 0.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;

    public static boolean hasAnyRollerPIDChanged() {
        return ROLLER_KS.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KV.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KA.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KP.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KI.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KD.hasChanged(ROLLER_TUNABLE_ID);
    }

    public static boolean hasAnyExtensionPIDChanged() {
        return EXTENSION_KS.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_KV.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_KA.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_KP.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_KI.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_KD.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_MOTION_MAGIC_EXPO_KV.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_MOTION_MAGIC_EXPO_KA.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_MOTION_MAGIC_CRUISE_VELOCITY.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_MOTION_MAGIC_ACCELERATION.hasChanged(EXTENSION_TUNABLE_ID)
                || EXTENSION_MOTION_MAGIC_JERK.hasChanged(EXTENSION_TUNABLE_ID);
    }
}
