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
        Logger.recordOutput("Indexer/IsRunning", isRunning());

        if (IndexerConstants.hasAnyConveyorPIDChanged()) {
            io.updateConveyorConfig();
        }
    }

    public void setConveyorVelocity(double velocityRadPerSec) {
        io.setConveyorVelocity(velocityRadPerSec);
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

    public boolean isRunning() {
        return Math.abs(inputs.conveyorVelocity) > VELOCITY_THRESHOLD_RAD_PER_SEC;
    }
}
