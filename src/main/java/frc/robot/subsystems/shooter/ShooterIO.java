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
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void setVelocity(double velocityRadPerSec) {}

    default void stop() {}

    default void updateShooterConfig() {}
}
