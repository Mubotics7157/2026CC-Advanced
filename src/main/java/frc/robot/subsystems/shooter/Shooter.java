package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/LeftVelocityRadPerSec", inputs.leftVelocity);
        Logger.recordOutput("Shooter/MiddleVelocityRadPerSec", inputs.middleVelocity);
        Logger.recordOutput("Shooter/RightVelocityRadPerSec", inputs.rightVelocity);
        Logger.recordOutput("Shooter/VelocitySetpointRadPerSec", inputs.velocitySetpoint);
        Logger.recordOutput(
                "Shooter/LeftVelocityErrorRadPerSec",
                inputs.velocitySetpoint - inputs.leftVelocity);
        Logger.recordOutput(
                "Shooter/MiddleVelocityErrorRadPerSec",
                inputs.velocitySetpoint - inputs.middleVelocity);
        Logger.recordOutput(
                "Shooter/RightVelocityErrorRadPerSec",
                inputs.velocitySetpoint - inputs.rightVelocity);
        Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
        Logger.recordOutput("Shooter/IsRunning", isRunning());

        if (ShooterConstants.hasAnyShooterPIDChanged()) {
            io.updateShooterConfig();
        }
    }

    public void setVelocity(double velocityRadPerSec) {
        io.setVelocity(velocityRadPerSec);
    }

    public void stop() {
        io.stop();
    }

    public double getLeftVelocityRadPerSec() {
        return inputs.leftVelocity;
    }

    public double getMiddleVelocityRadPerSec() {
        return inputs.middleVelocity;
    }

    public double getRightVelocityRadPerSec() {
        return inputs.rightVelocity;
    }

    public double getVelocitySetpointRadPerSec() {
        return inputs.velocitySetpoint;
    }

    public boolean atSetpoint() {
        if (Math.abs(inputs.velocitySetpoint) < ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC) {
            return false;
        }

        return Math.abs(inputs.velocitySetpoint - inputs.leftVelocity)
                        < ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC
                && Math.abs(inputs.velocitySetpoint - inputs.middleVelocity)
                        < ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC
                && Math.abs(inputs.velocitySetpoint - inputs.rightVelocity)
                        < ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC;
    }

    public boolean isRunning() {
        return Math.abs(inputs.leftVelocity) > ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC
                || Math.abs(inputs.middleVelocity) > ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC
                || Math.abs(inputs.rightVelocity) > ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC;
    }
}
