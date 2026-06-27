package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        // Roller Motor
        public double rollerCurrent = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerVelocitySetpoint = 0.0;

        // Rack-and-pinion extension motor
        public double extensionCurrent = 0.0;
        public double extensionVoltage = 0.0;
        public double extensionPosition = 0.0;
        public double extensionVelocity = 0.0;
        public double extensionPositionSetpoint = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerVelocity(double velocityRadPerSec) {}

    default void setExtensionVoltage(double voltage) {}

    default void setExtensionPosition(double positionMeters) {}

    default void stopRoller() {}

    default void stopExtensionMotor() {}

    default void updateRollerConfig() {}

    default void updateExtensionConfig() {}

    default void setExtensionEncoderPosition(double positionMeters) {}
}
