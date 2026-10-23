package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the three-wheel shooter. */
public final class ShooterConstants {

    private ShooterConstants() {}

    private static final int SHOOTER_TUNABLE_ID = "ShooterConstantsShooter".hashCode();

    public static final int LEFT_MOTOR_ID = 41;
    public static final int MIDDLE_MOTOR_ID = 42;
    public static final int RIGHT_MOTOR_ID = 43;
    public static final CANBus CAN_BUS = new CANBus("rio");

    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final double SHOOTER_STATOR_CURRENT_LIMIT = 120.0;

    public static final double VELOCITY_TOLERANCE_RAD_PER_SEC =
            Units.rotationsPerMinuteToRadiansPerSecond(100.0);
    public static final double SHOOTER_MAX_VELOCITY_RAD_PER_SEC =
            Units.rotationsPerMinuteToRadiansPerSecond(6000.0);
    public static final double SHOOTER_PEAK_TORQUE_CURRENT_AMPS = 120.0;
    public static final double HOOD_MIN_ANGLE_RAD = Units.degreesToRadians(20.0);
    public static final double HOOD_MAX_ANGLE_RAD = Units.degreesToRadians(65.0);

    /** Calibrated PWM travel limits for both hood servos. */
    public static final double HOOD_SERVO_MIN_POSITION = 0.01;

    public static final double HOOD_SERVO_MAX_POSITION = 0.77;

    /**
     * RoboRIO PWM ports for the left and right hood servos. Set these before enabling the real
     * robot. A negative channel intentionally leaves that servo disabled for safe bring-up.
     */
    public static final int LEFT_HOOD_SERVO_PWM_CHANNEL = 7;

    public static final int RIGHT_HOOD_SERVO_PWM_CHANNEL = 4;

    /** Conservative delay before feeding because hobby servos do not report measured position. */
    public static final double HOOD_SERVO_SETTLE_TIME_SECONDS = 0.25;

    public static double hoodAngleToServoPosition(double hoodAngleRad) {
        double normalizedAngle =
                MathUtil.inverseInterpolate(HOOD_MIN_ANGLE_RAD, HOOD_MAX_ANGLE_RAD, hoodAngleRad);
        return MathUtil.interpolate(
                HOOD_SERVO_MIN_POSITION,
                HOOD_SERVO_MAX_POSITION,
                MathUtil.clamp(normalizedAngle, 0.0, 1.0));
    }

    public static double servoPositionToHoodAngle(double servoPosition) {
        double normalizedPosition =
                MathUtil.inverseInterpolate(
                        HOOD_SERVO_MIN_POSITION, HOOD_SERVO_MAX_POSITION, servoPosition);
        return MathUtil.interpolate(
                HOOD_MIN_ANGLE_RAD,
                HOOD_MAX_ANGLE_RAD,
                MathUtil.clamp(normalizedPosition, 0.0, 1.0));
    }

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

    /** Large slot-1 gain with forward-only duty-cycle limits emulates BangBang control. */
    public static final LoggedTunableNumber BANG_BANG_KP =
            new LoggedTunableNumber("Shooter/BangBang/kP", 999999.0);
    public static final LoggedTunableNumber STARTUP_READY_RPM =
            new LoggedTunableNumber("Shooter/Phase/StartupReadyRPM", 80.0);
    public static final LoggedTunableNumber RECOVERY_READY_RPM =
            new LoggedTunableNumber("Shooter/Phase/RecoveryReadyRPM", 60.0);
    public static final LoggedTunableNumber BALL_ENTRY_DROP_RPM =
            new LoggedTunableNumber("Shooter/Phase/BallEntryDropRPM", 120.0);
    public static final LoggedTunableNumber BALL_PHASE_DURATION_SEC =
            new LoggedTunableNumber("Shooter/Phase/BallDurationSec", 0.06);
    public static final LoggedTunableNumber BALL_TORQUE_CURRENT_AMPS =
            new LoggedTunableNumber("Shooter/Phase/BallTorqueCurrentAmps", 29.0);

    public static final double STATUS_SIGNAL_UPDATE_FREQUENCY = 50.0;
    public static final double SHOOTER_MOI = 0.003;

    public static boolean hasAnyShooterPIDChanged() {
        return SHOOTER_KS.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KV.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KA.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KP.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KI.hasChanged(SHOOTER_TUNABLE_ID)
                || SHOOTER_KD.hasChanged(SHOOTER_TUNABLE_ID)
                || BANG_BANG_KP.hasChanged(SHOOTER_TUNABLE_ID);
    }
}
