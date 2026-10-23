package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Constants for the three-wheel shooter. */
public final class ShooterConstants {

    private ShooterConstants() {}

    private static final int SHOOTER_TUNABLE_ID = "ShooterConstantsShooter".hashCode();

    public enum Flywheel {
        LEFT("Left"),
        MIDDLE("Middle"),
        RIGHT("Right");

        public final LoggedTunableNumber kS;
        public final LoggedTunableNumber kV;
        public final LoggedTunableNumber kA;
        public final LoggedTunableNumber kP;
        public final LoggedTunableNumber kI;
        public final LoggedTunableNumber kD;

        Flywheel(String name) {
            String key = "Shooter/Flywheels/" + name + "/";
            kS = new LoggedTunableNumber(key + "kS", 0.0);
            kV = new LoggedTunableNumber(key + "kV", 0.12);
            kA = new LoggedTunableNumber(key + "kA", 0.0);
            kP = new LoggedTunableNumber(key + "kP", 0.5);
            kI = new LoggedTunableNumber(key + "kI", 2.0);
            kD = new LoggedTunableNumber(key + "kD", 0.0);
        }

        private boolean hasChanged() {
            boolean changed = false;
            changed |= kS.hasChanged(SHOOTER_TUNABLE_ID);
            changed |= kV.hasChanged(SHOOTER_TUNABLE_ID);
            changed |= kA.hasChanged(SHOOTER_TUNABLE_ID);
            changed |= kP.hasChanged(SHOOTER_TUNABLE_ID);
            changed |= kI.hasChanged(SHOOTER_TUNABLE_ID);
            changed |= kD.hasChanged(SHOOTER_TUNABLE_ID);
            return changed;
        }
    }

    public static final int LEFT_MOTOR_ID = 41;
    public static final int MIDDLE_MOTOR_ID = 42;
    public static final int RIGHT_MOTOR_ID = 43;
    public static final CANBus CAN_BUS = new CANBus("rio");

    public static final double SHOOTER_GEAR_RATIO = 1.0;
    public static final LoggedTunableNumber SHOOTER_STATOR_CURRENT_LIMIT_AMPS =
            new LoggedTunableNumber("Shooter/CurrentLimit/StatorAmps", 120.0);
    public static final LoggedTunableNumber SHOOTER_PEAK_TORQUE_CURRENT_AMPS =
            new LoggedTunableNumber("Shooter/CurrentLimit/TorqueAmps", 120.0);
    public static final LoggedTunableNumber VELOCITY_TOLERANCE_RPM =
            new LoggedTunableNumber("Shooter/Tuning/VelocityToleranceRPM", 100.0);
    public static final LoggedTunableNumber MAX_VELOCITY_RPM =
            new LoggedTunableNumber("Shooter/Tuning/MaxVelocityRPM", 6000.0);
    public static final LoggedTunableNumber TUNING_VELOCITY_RPM =
            new LoggedTunableNumber("Shooter/Tuning/TargetRPM", 2500.0);
    public static final LoggedTunableNumber TUNING_HOOD_ANGLE_DEG =
            new LoggedTunableNumber("Shooter/Tuning/HoodAngleDeg", 35.0);
    public static final LoggedTunableNumber CHARACTERIZATION_RAMP_VOLTS_PER_SEC =
            new LoggedTunableNumber("Shooter/Characterization/RampVoltsPerSec", 0.5);
    public static final LoggedTunableNumber CHARACTERIZATION_MAX_VOLTS =
            new LoggedTunableNumber("Shooter/Characterization/MaxVolts", 6.0);
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
        boolean changed = false;
        for (Flywheel flywheel : Flywheel.values()) {
            changed |= flywheel.hasChanged();
        }
        changed |= SHOOTER_STATOR_CURRENT_LIMIT_AMPS.hasChanged(SHOOTER_TUNABLE_ID);
        changed |= SHOOTER_PEAK_TORQUE_CURRENT_AMPS.hasChanged(SHOOTER_TUNABLE_ID);
        changed |= BANG_BANG_KP.hasChanged(SHOOTER_TUNABLE_ID);
        return changed;
    }
}
