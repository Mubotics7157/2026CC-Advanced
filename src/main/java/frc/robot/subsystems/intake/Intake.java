package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Rotating forebar intake. Position units are radians at the arm pivot. */
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
                "Intake/ArmPositionErrorRad", inputs.armPositionSetpoint - inputs.armPosition);
        Logger.recordOutput(
                "Intake/ArmAtSetpoint",
                isArmAtPosition(
                        inputs.armPositionSetpoint, IntakeConstants.ARM_POSITION_TOLERANCE_RAD));
        Logger.recordOutput(
                "Intake/RollerVelocityErrorRadPerSec",
                inputs.rollerVelocitySetpoint - inputs.rollerVelocity);
        Logger.recordOutput(
                "Intake/ArmInDeadzone",
                isArmAtPosition(
                        inputs.armPositionSetpoint, IntakeConstants.ARM_DEADZONE_TOLERANCE_RAD));
        if (IntakeConstants.hasAnyRollerPIDChanged()) io.updateRollerConfig();
        if (IntakeConstants.hasAnyArmPIDChanged()) io.updateArmConfig();
    }

    public void setRollerVelocity(double velocityRadPerSec) {
        io.setRollerVelocity(velocityRadPerSec);
    }

    public void setArmPosition(double positionRad) {
        if (isArmAtPosition(positionRad, IntakeConstants.ARM_DEADZONE_TOLERANCE_RAD))
            io.stopArmMotor();
        else io.setArmPosition(positionRad);
    }

    public void runArmVoltage(double volts) {
        io.setArmVoltage(volts);
    }

    public void deployArm() {
        setArmPosition(IntakeConstants.ARM_DEPLOYED_POSITION_RAD);
    }

    public void stowArm() {
        setArmPosition(IntakeConstants.ARM_STOWED_POSITION_RAD);
    }

    public void agitateArm() {
        setArmPosition(IntakeConstants.ARM_AGITATE_POSITION_RAD);
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

    public double getArmPositionRad() {
        return inputs.armPosition;
    }

    public double getArmVelocityRadPerSec() {
        return inputs.armVelocity;
    }

    public double getArmCurrent() {
        return inputs.armCurrent;
    }

    public double getArmVoltage() {
        return inputs.armVoltage;
    }

    public boolean isDeployed() {
        return isArmAtPosition(
                IntakeConstants.ARM_DEPLOYED_POSITION_RAD,
                IntakeConstants.ARM_POSITION_TOLERANCE_RAD);
    }

    public boolean isStowed() {
        return isArmAtPosition(
                IntakeConstants.ARM_STOWED_POSITION_RAD,
                IntakeConstants.ARM_POSITION_TOLERANCE_RAD);
    }

    public boolean isArmAtPosition(double targetRad, double toleranceRad) {
        return Math.abs(inputs.armPosition - targetRad) < toleranceRad;
    }

    public void stop() {
        io.stopRoller();
        io.stopArmMotor();
    }

    public void stopRoller() {
        io.stopRoller();
    }

    public void stopArm() {
        io.stopArmMotor();
    }

    public void resetArmEncoderToDefault() {
        io.setArmEncoderPosition(IntakeConstants.ARM_STOWED_POSITION_RAD);
    }

    public void setArmEncoderPosition(double positionRad) {
        io.setArmEncoderPosition(positionRad);
    }
}
