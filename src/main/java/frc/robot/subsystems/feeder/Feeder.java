package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private static final double VELOCITY_THRESHOLD_RAD_PER_SEC = 0.5;

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public Feeder(FeederIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        Logger.recordOutput("Feeder/VelocityRadPerSec", inputs.feederVelocity);
        Logger.recordOutput("Feeder/SetpointRadPerSec", inputs.feederVelocitySetpoint);
        Logger.recordOutput(
                "Feeder/VelocityErrorRadPerSec",
                inputs.feederVelocitySetpoint - inputs.feederVelocity);
        Logger.recordOutput("Feeder/IsRunning", isRunning());

        if (FeederConstants.hasAnyFeederPIDChanged()) {
            io.updateFeederConfig();
        }
    }

    public void setFeederVelocity(double velocityRadPerSec) {
        io.setFeederVelocity(velocityRadPerSec);
    }

    public void stop() {
        io.stop();
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

    public boolean isRunning() {
        return Math.abs(inputs.feederVelocity) > VELOCITY_THRESHOLD_RAD_PER_SEC;
    }
}
