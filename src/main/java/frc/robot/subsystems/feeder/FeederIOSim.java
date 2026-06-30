package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
    private static final double SIM_LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;

    private final DCMotorSim feederSim;

    private double feederAppliedVolts = 0.0;
    private double feederVelocitySetpoint = 0.0;

    public FeederIOSim() {
        feederSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60(1),
                                FeederConstants.FEEDER_MOI,
                                FeederConstants.FEEDER_GEAR_RATIO),
                        DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederSim.update(SIM_LOOP_PERIOD);

        inputs.feederVelocity = feederSim.getAngularVelocityRadPerSec();
        inputs.feederVoltage = feederAppliedVolts;
        inputs.feederCurrent = feederSim.getCurrentDrawAmps();
        inputs.feederVelocitySetpoint = feederVelocitySetpoint;
    }

    @Override
    public void setFeederVelocity(double velocityRadPerSec) {
        feederVelocitySetpoint = velocityRadPerSec;
        double error = velocityRadPerSec - feederSim.getAngularVelocityRadPerSec();
        feederAppliedVolts = MathUtil.clamp(error * 0.1, -MAX_VOLTAGE, MAX_VOLTAGE);
        feederSim.setInputVoltage(feederAppliedVolts);
    }

    @Override
    public void stop() {
        feederVelocitySetpoint = 0.0;
        feederAppliedVolts = 0.0;
        feederSim.setInputVoltage(0.0);
    }
}
