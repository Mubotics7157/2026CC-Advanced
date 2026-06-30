package frc.robot.subsystems.feeder;

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

public class FeederIOReal implements FeederIO {
    private final TalonFX feederMotor;
    private final VelocityTorqueCurrentFOC feederVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);

    private final StatusSignal<AngularVelocity> feederVelocity;
    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederCurrent;

    private double feederVelocitySetpoint = 0.0;

    public FeederIOReal() {
        feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID, FeederConstants.CAN_BUS);

        var feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.FEEDER_STATOR_CURRENT_LIMIT;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.Feedback.SensorToMechanismRatio = FeederConstants.FEEDER_GEAR_RATIO;
        applyFeederGains(feederConfig.Slot0);
        feederMotor.getConfigurator().apply(feederConfig);

        feederVelocity = feederMotor.getVelocity();
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederCurrent = feederMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                FeederConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                feederVelocity,
                feederAppliedVolts,
                feederCurrent);

        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(feederVelocity, feederAppliedVolts, feederCurrent);

        inputs.feederVelocity = Units.rotationsToRadians(feederVelocity.getValueAsDouble());
        inputs.feederVoltage = feederAppliedVolts.getValueAsDouble();
        inputs.feederCurrent = feederCurrent.getValueAsDouble();
        inputs.feederVelocitySetpoint = feederVelocitySetpoint;
    }

    @Override
    public void setFeederVelocity(double velocityRadPerSec) {
        feederVelocitySetpoint = velocityRadPerSec;
        feederMotor.setControl(
                feederVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void stop() {
        feederVelocitySetpoint = 0.0;
        feederMotor.stopMotor();
    }

    @Override
    public void updateFeederConfig() {
        var slot0 = new Slot0Configs();
        applyFeederGains(slot0);
        feederMotor.getConfigurator().apply(slot0);
    }

    private static void applyFeederGains(Slot0Configs slot0) {
        slot0.kS = FeederConstants.FEEDER_KS.get();
        slot0.kV = FeederConstants.FEEDER_KV.get() * FeederConstants.FEEDER_GEAR_RATIO;
        slot0.kA = FeederConstants.FEEDER_KA.get() * FeederConstants.FEEDER_GEAR_RATIO;
        slot0.kP = FeederConstants.FEEDER_KP.get() * FeederConstants.FEEDER_GEAR_RATIO;
        slot0.kI = FeederConstants.FEEDER_KI.get() * FeederConstants.FEEDER_GEAR_RATIO;
        slot0.kD = FeederConstants.FEEDER_KD.get() * FeederConstants.FEEDER_GEAR_RATIO;
    }
}
