package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the rotating forebar intake and its rollers. */
public final class IntakeConstants {
    private IntakeConstants() {}

    private static final int ROLLER_TUNABLE_ID = "IntakeConstantsRoller".hashCode();
    private static final int ARM_TUNABLE_ID = "IntakeConstantsArm".hashCode();

    public static final int ROLLER_MOTOR_ID = 22;
    public static final int LEFT_ROLLER_MOTOR_ID = 23;
    public static final int ARM_MOTOR_ID = 21;
    public static final CANBus CAN_BUS = new CANBus("swerve");

    public static final double ROLLER_GEAR_RATIO = 1.42857143;
    public static final double ARM_GEAR_RATIO = 47.1428571;

    // Replace these preliminary positions, gains, and motion constraints after mechanism tuning.
    public static final double ARM_STOWED_POSITION_RAD = 0.0;
    public static final double ARM_DEPLOYED_POSITION_RAD = -2.2;
    public static final double ARM_AGITATE_POSITION_RAD = -0.3;
    public static final double ARM_POSITION_TOLERANCE_RAD = 0.15;
    public static final double ARM_DEADZONE_TOLERANCE_RAD = 0.15;

    public static final double ROLLER_STATOR_CURRENT_LIMIT = 40.0;
    public static final double ARM_STATOR_CURRENT_LIMIT = 40.0;

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

    public static final LoggedTunableNumber ARM_KS = new LoggedTunableNumber("Intake/Arm/kS", 0.0);
    public static final LoggedTunableNumber ARM_KV = new LoggedTunableNumber("Intake/Arm/kV", 0.0);
    public static final LoggedTunableNumber ARM_KA = new LoggedTunableNumber("Intake/Arm/kA", 0.0);
    public static final LoggedTunableNumber ARM_KG =
            new LoggedTunableNumber("Intake/Arm/kG", 0.212);
    public static final LoggedTunableNumber ARM_KP = new LoggedTunableNumber("Intake/Arm/kP", 53.0);
    public static final LoggedTunableNumber ARM_KI = new LoggedTunableNumber("Intake/Arm/kI", 0.0);
    public static final LoggedTunableNumber ARM_KD = new LoggedTunableNumber("Intake/Arm/kD", 4.25);
    public static final LoggedTunableNumber ARM_GRAVITY_OFFSET_ROT =
            new LoggedTunableNumber("Intake/Arm/GravityOffsetRot", 0.0);

    public static final LoggedTunableNumber ARM_MOTION_MAGIC_EXPO_KV =
            new LoggedTunableNumber("Intake/Arm/MMExpo_kV", 0.12);
    public static final LoggedTunableNumber ARM_MOTION_MAGIC_EXPO_KA =
            new LoggedTunableNumber("Intake/Arm/MMExpo_kA", 0.1);
    public static final LoggedTunableNumber ARM_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC =
            new LoggedTunableNumber("Intake/Arm/MMCruiseVelRadPerSec", 25.0);
    public static final LoggedTunableNumber ARM_MOTION_MAGIC_ACCELERATION_RAD_PER_SEC_SQ =
            new LoggedTunableNumber("Intake/Arm/MMAccelRadPerSecSq", 0.0);
    public static final LoggedTunableNumber ARM_MOTION_MAGIC_JERK_RAD_PER_SEC_CU =
            new LoggedTunableNumber("Intake/Arm/MMJerkRadPerSecCubed", 0.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;

    public static boolean hasAnyRollerPIDChanged() {
        return ROLLER_KS.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KV.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KA.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KP.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KI.hasChanged(ROLLER_TUNABLE_ID)
                || ROLLER_KD.hasChanged(ROLLER_TUNABLE_ID);
    }

    public static boolean hasAnyArmPIDChanged() {
        return ARM_KS.hasChanged(ARM_TUNABLE_ID)
                || ARM_KV.hasChanged(ARM_TUNABLE_ID)
                || ARM_KA.hasChanged(ARM_TUNABLE_ID)
                || ARM_KG.hasChanged(ARM_TUNABLE_ID)
                || ARM_KP.hasChanged(ARM_TUNABLE_ID)
                || ARM_KI.hasChanged(ARM_TUNABLE_ID)
                || ARM_KD.hasChanged(ARM_TUNABLE_ID)
                || ARM_GRAVITY_OFFSET_ROT.hasChanged(ARM_TUNABLE_ID)
                || ARM_MOTION_MAGIC_EXPO_KV.hasChanged(ARM_TUNABLE_ID)
                || ARM_MOTION_MAGIC_EXPO_KA.hasChanged(ARM_TUNABLE_ID)
                || ARM_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC.hasChanged(ARM_TUNABLE_ID)
                || ARM_MOTION_MAGIC_ACCELERATION_RAD_PER_SEC_SQ.hasChanged(ARM_TUNABLE_ID)
                || ARM_MOTION_MAGIC_JERK_RAD_PER_SEC_CU.hasChanged(ARM_TUNABLE_ID);
    }
}
