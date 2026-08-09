package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX conveyorMotor;
    private final TalonFX feederMotor;
    private final VelocityTorqueCurrentFOC conveyorVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC feederVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);

    private final StatusSignal<AngularVelocity> conveyorVelocity;
    private final StatusSignal<Voltage> conveyorAppliedVolts;
    private final StatusSignal<Current> conveyorCurrent;
    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederCurrent;

    private double conveyorVelocitySetpoint = 0.0;
    private double feederVelocitySetpoint = 0.0;

    public IndexerIOReal() {
        conveyorMotor = new TalonFX(IndexerConstants.CONVEYOR_MOTOR_ID, IndexerConstants.CAN_BUS);
        feederMotor = new TalonFX(IndexerConstants.FEEDER_MOTOR_ID, IndexerConstants.CAN_BUS);

        var conveyorConfig = new TalonFXConfiguration();
        conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        conveyorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        conveyorConfig.CurrentLimits.StatorCurrentLimit =
                IndexerConstants.CONVEYOR_STATOR_CURRENT_LIMIT;
        conveyorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        conveyorConfig.Feedback.SensorToMechanismRatio = IndexerConstants.CONVEYOR_GEAR_RATIO;
        applyConveyorGains(conveyorConfig.Slot0);
        conveyorMotor.getConfigurator().apply(conveyorConfig);

        var feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.CurrentLimits.StatorCurrentLimit =
                IndexerConstants.FEEDER_STATOR_CURRENT_LIMIT;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.Feedback.SensorToMechanismRatio = IndexerConstants.FEEDER_GEAR_RATIO;
        applyFeederGains(feederConfig.Slot0);
        feederMotor.getConfigurator().apply(feederConfig);

        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorAppliedVolts = conveyorMotor.getMotorVoltage();
        conveyorCurrent = conveyorMotor.getStatorCurrent();
        feederVelocity = feederMotor.getVelocity();
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederCurrent = feederMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IndexerConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                conveyorVelocity,
                conveyorAppliedVolts,
                conveyorCurrent,
                feederVelocity,
                feederAppliedVolts,
                feederCurrent);

        conveyorMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                conveyorVelocity,
                conveyorAppliedVolts,
                conveyorCurrent,
                feederVelocity,
                feederAppliedVolts,
                feederCurrent);

        inputs.conveyorVelocity = Units.rotationsToRadians(conveyorVelocity.getValueAsDouble());
        inputs.conveyorVoltage = conveyorAppliedVolts.getValueAsDouble();
        inputs.conveyorCurrent = conveyorCurrent.getValueAsDouble();
        inputs.conveyorVelocitySetpoint = conveyorVelocitySetpoint;
        inputs.feederVelocity = Units.rotationsToRadians(feederVelocity.getValueAsDouble());
        inputs.feederVoltage = feederAppliedVolts.getValueAsDouble();
        inputs.feederCurrent = feederCurrent.getValueAsDouble();
        inputs.feederVelocitySetpoint = feederVelocitySetpoint;
    }

    @Override
    public void setConveyorVelocity(double velocityRadPerSec) {
        conveyorVelocitySetpoint = velocityRadPerSec;
        conveyorMotor.setControl(
                conveyorVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void setFeederVelocity(double velocityRadPerSec) {
        feederVelocitySetpoint = velocityRadPerSec;
        feederMotor.setControl(
                feederVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void stopConveyor() {
        conveyorVelocitySetpoint = 0.0;
        conveyorMotor.stopMotor();
    }

    @Override
    public void stopFeeder() {
        feederVelocitySetpoint = 0.0;
        feederMotor.stopMotor();
    }

    @Override
    public void stop() {
        stopConveyor();
        stopFeeder();
    }

    @Override
    public void updateConveyorConfig() {
        var slot0 = new Slot0Configs();
        applyConveyorGains(slot0);
        conveyorMotor.getConfigurator().apply(slot0);
    }

    @Override
    public void updateFeederConfig() {
        var slot0 = new Slot0Configs();
        applyFeederGains(slot0);
        feederMotor.getConfigurator().apply(slot0);
    }

    private static void applyConveyorGains(Slot0Configs slot0) {
        slot0.kS = IndexerConstants.CONVEYOR_KS.get();
        slot0.kV = IndexerConstants.CONVEYOR_KV.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kA = IndexerConstants.CONVEYOR_KA.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kP = IndexerConstants.CONVEYOR_KP.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kI = IndexerConstants.CONVEYOR_KI.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kD = IndexerConstants.CONVEYOR_KD.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
    }

    private static void applyFeederGains(Slot0Configs slot0) {
        slot0.kS = IndexerConstants.FEEDER_KS.get();
        slot0.kV = IndexerConstants.FEEDER_KV.get() * IndexerConstants.FEEDER_GEAR_RATIO;
        slot0.kA = IndexerConstants.FEEDER_KA.get() * IndexerConstants.FEEDER_GEAR_RATIO;
        slot0.kP = IndexerConstants.FEEDER_KP.get() * IndexerConstants.FEEDER_GEAR_RATIO;
        slot0.kI = IndexerConstants.FEEDER_KI.get() * IndexerConstants.FEEDER_GEAR_RATIO;
        slot0.kD = IndexerConstants.FEEDER_KD.get() * IndexerConstants.FEEDER_GEAR_RATIO;
    }
}
