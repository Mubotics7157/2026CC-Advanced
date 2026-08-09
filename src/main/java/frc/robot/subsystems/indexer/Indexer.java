package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD_RAD_PER_SEC = 0.5;

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        Logger.recordOutput("Indexer/ConveyorVelocityRadPerSec", inputs.conveyorVelocity);
        Logger.recordOutput("Indexer/ConveyorSetpointRadPerSec", inputs.conveyorVelocitySetpoint);
        Logger.recordOutput(
                "Indexer/ConveyorVelocityErrorRadPerSec",
                inputs.conveyorVelocitySetpoint - inputs.conveyorVelocity);
        Logger.recordOutput("Indexer/ConveyorRunning", isConveyorRunning());
        Logger.recordOutput("Indexer/FeederVelocityRadPerSec", inputs.feederVelocity);
        Logger.recordOutput("Indexer/FeederSetpointRadPerSec", inputs.feederVelocitySetpoint);
        Logger.recordOutput(
                "Indexer/FeederVelocityErrorRadPerSec",
                inputs.feederVelocitySetpoint - inputs.feederVelocity);
        Logger.recordOutput("Indexer/FeederRunning", isFeederRunning());
        Logger.recordOutput("Indexer/IsRunning", isRunning());

        if (IndexerConstants.hasAnyConveyorPIDChanged()) {
            io.updateConveyorConfig();
        }
        if (IndexerConstants.hasAnyFeederPIDChanged()) {
            io.updateFeederConfig();
        }
    }

    public void setConveyorVelocity(double velocityRadPerSec) {
        io.setConveyorVelocity(velocityRadPerSec);
    }

    public void setFeederVelocity(double velocityRadPerSec) {
        io.setFeederVelocity(velocityRadPerSec);
    }

    public void stopConveyor() {
        io.stopConveyor();
    }

    public void stopFeeder() {
        io.stopFeeder();
    }

    public void stop() {
        io.stop();
    }

    public double getConveyorVelocityRadPerSec() {
        return inputs.conveyorVelocity;
    }

    public double getConveyorCurrent() {
        return inputs.conveyorCurrent;
    }

    public double getConveyorVoltage() {
        return inputs.conveyorVoltage;
    }

    public double getFeederVelocityRadPerSec() {
        return inputs.feederVelocity;
    }

    public double getFeederCurrent() {
        return inputs.feederCurrent;
    }

    public double getFeederVoltage() {
        return inputs.feederVoltage;
    }

    public boolean isConveyorRunning() {
        return Math.abs(inputs.conveyorVelocity) > VELOCITY_THRESHOLD_RAD_PER_SEC;
    }

    public boolean isFeederRunning() {
        return Math.abs(inputs.feederVelocity) > VELOCITY_THRESHOLD_RAD_PER_SEC;
    }

    public boolean isRunning() {
        return isConveyorRunning() || isFeederRunning();
    }
}
