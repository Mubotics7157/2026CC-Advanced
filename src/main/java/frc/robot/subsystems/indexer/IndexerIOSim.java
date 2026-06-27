package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
    private static final double SIM_LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;

    private final DCMotorSim conveyorSim;

    private double conveyorAppliedVolts = 0.0;
    private double conveyorVelocitySetpoint = 0.0;

    public IndexerIOSim() {
        conveyorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60(1),
                                IndexerConstants.CONVEYOR_MOI,
                                IndexerConstants.CONVEYOR_GEAR_RATIO),
                        DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        conveyorSim.update(SIM_LOOP_PERIOD);

        inputs.conveyorVelocity = conveyorSim.getAngularVelocityRadPerSec();
        inputs.conveyorVoltage = conveyorAppliedVolts;
        inputs.conveyorCurrent = conveyorSim.getCurrentDrawAmps();
        inputs.conveyorVelocitySetpoint = conveyorVelocitySetpoint;
    }

    @Override
    public void setConveyorVelocity(double velocityRadPerSec) {
        conveyorVelocitySetpoint = velocityRadPerSec;
        double error = velocityRadPerSec - conveyorSim.getAngularVelocityRadPerSec();
        conveyorAppliedVolts = MathUtil.clamp(error * 0.1, -MAX_VOLTAGE, MAX_VOLTAGE);
        conveyorSim.setInputVoltage(conveyorAppliedVolts);
    }

    @Override
    public void stop() {
        conveyorVelocitySetpoint = 0.0;
        conveyorAppliedVolts = 0.0;
        conveyorSim.setInputVoltage(0.0);
    }
}
