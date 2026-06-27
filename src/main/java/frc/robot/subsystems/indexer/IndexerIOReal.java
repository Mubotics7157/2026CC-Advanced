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
    private final VelocityTorqueCurrentFOC conveyorVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);

    private final StatusSignal<AngularVelocity> conveyorVelocity;
    private final StatusSignal<Voltage> conveyorAppliedVolts;
    private final StatusSignal<Current> conveyorCurrent;

    private double conveyorVelocitySetpoint = 0.0;

    public IndexerIOReal() {
        conveyorMotor = new TalonFX(IndexerConstants.CONVEYOR_MOTOR_ID, IndexerConstants.CAN_BUS);

        var conveyorConfig = new TalonFXConfiguration();
        conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        conveyorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        conveyorConfig.CurrentLimits.StatorCurrentLimit =
                IndexerConstants.CONVEYOR_STATOR_CURRENT_LIMIT;
        conveyorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        conveyorConfig.Feedback.SensorToMechanismRatio = IndexerConstants.CONVEYOR_GEAR_RATIO;
        applyConveyorGains(conveyorConfig.Slot0);
        conveyorMotor.getConfigurator().apply(conveyorConfig);

        conveyorVelocity = conveyorMotor.getVelocity();
        conveyorAppliedVolts = conveyorMotor.getMotorVoltage();
        conveyorCurrent = conveyorMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IndexerConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                conveyorVelocity,
                conveyorAppliedVolts,
                conveyorCurrent);

        conveyorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(conveyorVelocity, conveyorAppliedVolts, conveyorCurrent);

        inputs.conveyorVelocity = Units.rotationsToRadians(conveyorVelocity.getValueAsDouble());
        inputs.conveyorVoltage = conveyorAppliedVolts.getValueAsDouble();
        inputs.conveyorCurrent = conveyorCurrent.getValueAsDouble();
        inputs.conveyorVelocitySetpoint = conveyorVelocitySetpoint;
    }

    @Override
    public void setConveyorVelocity(double velocityRadPerSec) {
        conveyorVelocitySetpoint = velocityRadPerSec;
        conveyorMotor.setControl(
                conveyorVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void stop() {
        conveyorVelocitySetpoint = 0.0;
        conveyorMotor.stopMotor();
    }

    @Override
    public void updateConveyorConfig() {
        var slot0 = new Slot0Configs();
        applyConveyorGains(slot0);
        conveyorMotor.getConfigurator().apply(slot0);
    }

    private static void applyConveyorGains(Slot0Configs slot0) {
        slot0.kS = IndexerConstants.CONVEYOR_KS.get();
        slot0.kV = IndexerConstants.CONVEYOR_KV.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kA = IndexerConstants.CONVEYOR_KA.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kP = IndexerConstants.CONVEYOR_KP.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kI = IndexerConstants.CONVEYOR_KI.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
        slot0.kD = IndexerConstants.CONVEYOR_KD.get() * IndexerConstants.CONVEYOR_GEAR_RATIO;
    }
}
