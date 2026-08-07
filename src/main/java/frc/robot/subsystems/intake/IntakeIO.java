package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rollerCurrent = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerPosition = 0.0;
        public double rollerVelocity = 0.0;
        public double rollerVelocitySetpoint = 0.0;
        public double armCurrent = 0.0;
        public double armVoltage = 0.0;
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double armPositionSetpoint = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerVelocity(double velocityRadPerSec) {}

    default void setArmVoltage(double voltage) {}

    default void setArmPosition(double positionRad) {}

    default void stopRoller() {}

    default void stopArmMotor() {}

    default void updateRollerConfig() {}

    default void updateArmConfig() {}

    default void setArmEncoderPosition(double positionRad) {}
}
