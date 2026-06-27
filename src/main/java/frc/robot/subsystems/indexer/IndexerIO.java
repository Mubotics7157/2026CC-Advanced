package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double conveyorVelocity = 0.0;
        public double conveyorVoltage = 0.0;
        public double conveyorCurrent = 0.0;
        public double conveyorVelocitySetpoint = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setConveyorVelocity(double velocityRadPerSec) {}

    default void stop() {}

    default void updateConveyorConfig() {}
}
