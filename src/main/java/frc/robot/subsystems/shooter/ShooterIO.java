package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double leftVelocity = 0.0;
        public double middleVelocity = 0.0;
        public double rightVelocity = 0.0;

        public double leftVoltage = 0.0;
        public double middleVoltage = 0.0;
        public double rightVoltage = 0.0;

        public double leftCurrent = 0.0;
        public double middleCurrent = 0.0;
        public double rightCurrent = 0.0;

        public double velocitySetpoint = 0.0;

        /** Commanded servo positions; PWM servos do not provide mechanism-position feedback. */
        public double leftHoodServoPosition = ShooterConstants.HOOD_SERVO_MIN_POSITION;

        public double rightHoodServoPosition = ShooterConstants.HOOD_SERVO_MIN_POSITION;
        public double hoodServoSetpoint = ShooterConstants.HOOD_SERVO_MIN_POSITION;
        public boolean hoodAtSetpoint = false;
        public boolean hoodHardwareConfigured = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void setVelocity(double velocityRadPerSec) {}

    default void setHoodServoPosition(double position) {}

    default void stop() {}

    default void updateShooterConfig() {}
}
