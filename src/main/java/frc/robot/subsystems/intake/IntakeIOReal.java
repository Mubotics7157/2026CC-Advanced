package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private final VelocityTorqueCurrentFOC rollerVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);
    private final MotionMagicExpoTorqueCurrentFOC extensionPositionRequest =
            new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final VoltageOut extensionVoltageRequest = new VoltageOut(0.0);

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerCurrent;

    private final StatusSignal<Angle> extensionPosition;
    private final StatusSignal<AngularVelocity> extensionVelocity;
    private final StatusSignal<Voltage> extensionAppliedVolts;
    private final StatusSignal<Current> extensionCurrent;

    private double rollerVelocitySetpoint = 0.0;
    private double extensionPositionSetpoint = 0.0;

    public IntakeIOReal() {
        rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
        extensionMotor = new TalonFX(IntakeConstants.EXTENSION_MOTOR_ID, IntakeConstants.CAN_BUS);

        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEAR_RATIO;
        applyRollerGains(rollerConfig.Slot0);
        rollerMotor.getConfigurator().apply(rollerConfig);

        var extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extensionConfig.CurrentLimits.StatorCurrentLimit =
                IntakeConstants.EXTENSION_STATOR_CURRENT_LIMIT;
        extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        extensionConfig.Feedback.SensorToMechanismRatio =
                IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
        applyExtensionGains(extensionConfig.Slot0);
        applyExtensionMotionMagic(extensionConfig.MotionMagic);
        extensionMotor.getConfigurator().apply(extensionConfig);

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();

        extensionPosition = extensionMotor.getPosition();
        extensionVelocity = extensionMotor.getVelocity();
        extensionAppliedVolts = extensionMotor.getMotorVoltage();
        extensionCurrent = extensionMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                extensionPosition,
                extensionVelocity,
                extensionAppliedVolts,
                extensionCurrent);
        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                extensionPosition,
                extensionVelocity,
                extensionAppliedVolts,
                extensionCurrent);

        inputs.rollerPosition = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
        inputs.rollerVelocity = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
        inputs.rollerVoltage = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrent.getValueAsDouble();

        inputs.extensionPosition = pinionRotationsToMeters(extensionPosition.getValueAsDouble());
        inputs.extensionVelocity = pinionRotationsToMeters(extensionVelocity.getValueAsDouble());
        inputs.extensionVoltage = extensionAppliedVolts.getValueAsDouble();
        inputs.extensionCurrent = extensionCurrent.getValueAsDouble();

        inputs.rollerVelocitySetpoint = rollerVelocitySetpoint;
        inputs.extensionPositionSetpoint = extensionPositionSetpoint;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        rollerVelocitySetpoint = velocityRadPerSec;
        rollerMotor.setControl(
                rollerVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extensionMotor.setControl(
                extensionVoltageRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setExtensionPosition(double positionMeters) {
        extensionPositionSetpoint = positionMeters;
        extensionMotor.setControl(
                extensionPositionRequest.withPosition(metersToPinionRotations(positionMeters)));
    }

    @Override
    public void stopRoller() {
        rollerVelocitySetpoint = 0.0;
        rollerMotor.stopMotor();
    }

    @Override
    public void stopExtensionMotor() {
        extensionMotor.stopMotor();
    }

    @Override
    public void updateRollerConfig() {
        var slot0 = new Slot0Configs();
        applyRollerGains(slot0);
        rollerMotor.getConfigurator().apply(slot0);
    }

    @Override
    public void updateExtensionConfig() {
        var slot0 = new Slot0Configs();
        applyExtensionGains(slot0);
        extensionMotor.getConfigurator().apply(slot0);

        var motionMagic = new MotionMagicConfigs();
        applyExtensionMotionMagic(motionMagic);
        extensionMotor.getConfigurator().apply(motionMagic);
    }

    @Override
    public void setExtensionEncoderPosition(double positionMeters) {
        extensionMotor.setPosition(metersToPinionRotations(positionMeters));
    }

    private static void applyRollerGains(Slot0Configs slot0) {
        slot0.kS = IntakeConstants.ROLLER_KS.get();
        slot0.kV = IntakeConstants.ROLLER_KV.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kA = IntakeConstants.ROLLER_KA.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kP = IntakeConstants.ROLLER_KP.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kI = IntakeConstants.ROLLER_KI.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kD = IntakeConstants.ROLLER_KD.get() * IntakeConstants.ROLLER_GEAR_RATIO;
    }

    private static void applyExtensionGains(Slot0Configs slot0) {
        slot0.kS = IntakeConstants.EXTENSION_KS.get();
        slot0.kV =
                IntakeConstants.EXTENSION_KV.get()
                        * IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
        slot0.kA =
                IntakeConstants.EXTENSION_KA.get()
                        * IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
        slot0.kP =
                IntakeConstants.EXTENSION_KP.get()
                        * IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
        slot0.kI =
                IntakeConstants.EXTENSION_KI.get()
                        * IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
        slot0.kD =
                IntakeConstants.EXTENSION_KD.get()
                        * IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO;
    }

    private static void applyExtensionMotionMagic(MotionMagicConfigs motionMagic) {
        motionMagic.MotionMagicExpo_kV = IntakeConstants.EXTENSION_MOTION_MAGIC_EXPO_KV.get();
        motionMagic.MotionMagicExpo_kA = IntakeConstants.EXTENSION_MOTION_MAGIC_EXPO_KA.get();
        motionMagic.MotionMagicCruiseVelocity =
                metersToPinionRotations(
                        IntakeConstants.EXTENSION_MOTION_MAGIC_CRUISE_VELOCITY.get());
        motionMagic.MotionMagicAcceleration =
                metersToPinionRotations(IntakeConstants.EXTENSION_MOTION_MAGIC_ACCELERATION.get());
        motionMagic.MotionMagicJerk =
                metersToPinionRotations(IntakeConstants.EXTENSION_MOTION_MAGIC_JERK.get());
    }

    private static double metersToPinionRotations(double meters) {
        return meters * IntakeConstants.EXTENSION_PINION_ROTATIONS_PER_METER;
    }

    private static double pinionRotationsToMeters(double pinionRotations) {
        return pinionRotations / IntakeConstants.EXTENSION_PINION_ROTATIONS_PER_METER;
    }
}
