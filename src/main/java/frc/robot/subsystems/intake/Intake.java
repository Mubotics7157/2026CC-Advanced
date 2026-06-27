package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput(
                "Intake/ExtensionPositionErrorMeters",
                inputs.extensionPositionSetpoint - inputs.extensionPosition);
        Logger.recordOutput(
                "Intake/ExtensionAtSetpoint",
                isExtensionAtPosition(
                        inputs.extensionPositionSetpoint,
                        IntakeConstants.EXTENSION_POSITION_TOLERANCE_METERS));
        Logger.recordOutput(
                "Intake/RollerVelocityErrorRadPerSec",
                inputs.rollerVelocitySetpoint - inputs.rollerVelocity);
        Logger.recordOutput("Intake/CurrentExtensionPositionMeters", inputs.extensionPosition);
        Logger.recordOutput(
                "Intake/ExtensionInDeadzone",
                Math.abs(inputs.extensionPositionSetpoint - inputs.extensionPosition)
                        < IntakeConstants.EXTENSION_DEADZONE_TOLERANCE_METERS);

        if (IntakeConstants.hasAnyRollerPIDChanged()) {
            io.updateRollerConfig();
        }
        if (IntakeConstants.hasAnyExtensionPIDChanged()) {
            io.updateExtensionConfig();
        }
    }

    public void setRollerVelocity(double velocityRadPerSec) {
        io.setRollerVelocity(velocityRadPerSec);
    }

    public void setExtensionPosition(double positionMeters) {
        if (isExtensionAtPosition(
                positionMeters, IntakeConstants.EXTENSION_DEADZONE_TOLERANCE_METERS)) {
            io.stopExtensionMotor();
        } else {
            io.setExtensionPosition(positionMeters);
        }
    }

    public void runExtensionVoltage(double volts) {
        io.setExtensionVoltage(volts);
    }

    public void extend() {
        setExtensionPosition(IntakeConstants.EXTENSION_EXTENDED_POSITION_METERS);
    }

    public void retract() {
        setExtensionPosition(IntakeConstants.EXTENSION_RETRACTED_POSITION_METERS);
    }

    public double getRollerPositionRad() {
        return inputs.rollerPosition;
    }

    public double getRollerVelocityRadPerSec() {
        return inputs.rollerVelocity;
    }

    public double getRollerCurrent() {
        return inputs.rollerCurrent;
    }

    public double getRollerVoltage() {
        return inputs.rollerVoltage;
    }

    public double getExtensionPositionMeters() {
        return inputs.extensionPosition;
    }

    public double getExtensionVelocityMetersPerSec() {
        return inputs.extensionVelocity;
    }

    public double getExtensionCurrent() {
        return inputs.extensionCurrent;
    }

    public double getExtensionVoltage() {
        return inputs.extensionVoltage;
    }

    public boolean isExtended() {
        return isExtensionAtPosition(
                IntakeConstants.EXTENSION_EXTENDED_POSITION_METERS,
                IntakeConstants.EXTENSION_POSITION_TOLERANCE_METERS);
    }

    public boolean isRetracted() {
        return isExtensionAtPosition(
                IntakeConstants.EXTENSION_RETRACTED_POSITION_METERS,
                IntakeConstants.EXTENSION_POSITION_TOLERANCE_METERS);
    }

    public boolean isExtensionAtPosition(double targetMeters, double toleranceMeters) {
        return Math.abs(inputs.extensionPosition - targetMeters) < toleranceMeters;
    }

    public void stop() {
        io.stopRoller();
        io.stopExtensionMotor();
    }

    public void stopRoller() {
        io.stopRoller();
    }

    public void stopExtension() {
        io.stopExtensionMotor();
    }

    public void resetExtensionEncoderToDefault() {
        io.setExtensionEncoderPosition(IntakeConstants.EXTENSION_RETRACTED_POSITION_METERS);
    }

    public void setExtensionEncoderPosition(double positionMeters) {
        io.setExtensionEncoderPosition(positionMeters);
    }
}
