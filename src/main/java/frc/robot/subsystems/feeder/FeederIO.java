package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public double feederVelocity = 0.0;
        public double feederVoltage = 0.0;
        public double feederCurrent = 0.0;
        public double feederVelocitySetpoint = 0.0;
    }

    default void updateInputs(FeederIOInputs inputs) {}

    default void setFeederVelocity(double velocityRadPerSec) {}

    default void stop() {}

    default void updateFeederConfig() {}
}
