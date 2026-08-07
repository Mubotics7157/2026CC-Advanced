package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** CTRE hardware implementation for the forebar intake. */
public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerMotor =
            new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final TalonFX leftRollerMotor =
            new TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final TalonFX armMotor =
            new TalonFX(IntakeConstants.ARM_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final VelocityTorqueCurrentFOC rollerVelocityRequest =
            new VelocityTorqueCurrentFOC(0.0);
    private final MotionMagicExpoTorqueCurrentFOC armPositionRequest =
            new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final VoltageOut armVoltageRequest = new VoltageOut(0.0);
    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Voltage> armAppliedVolts;
    private final StatusSignal<Current> armCurrent;
    private double rollerVelocitySetpoint;
    private double armPositionSetpoint;

    public IntakeIOReal() {
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEAR_RATIO;
        applyRollerGains(rollerConfig.Slot0);
        rollerMotor.getConfigurator().apply(rollerConfig);
        leftRollerMotor.setControl(
                new Follower(IntakeConstants.ROLLER_MOTOR_ID, MotorAlignmentValue.Opposed));

        var armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.ARM_STATOR_CURRENT_LIMIT;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.Feedback.SensorToMechanismRatio = IntakeConstants.ARM_GEAR_RATIO;
        applyArmGains(armConfig.Slot0);
        applyArmMotionMagic(armConfig.MotionMagic);
        armMotor.getConfigurator().apply(armConfig);

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getStatorCurrent();
        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armCurrent = armMotor.getStatorCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
                IntakeConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                armPosition,
                armVelocity,
                armAppliedVolts,
                armCurrent);
        rollerMotor.optimizeBusUtilization();
        leftRollerMotor.optimizeBusUtilization();
        armMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                rollerPosition,
                rollerVelocity,
                rollerAppliedVolts,
                rollerCurrent,
                armPosition,
                armVelocity,
                armAppliedVolts,
                armCurrent);
        inputs.rollerPosition = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
        inputs.rollerVelocity = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
        inputs.rollerVoltage = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrent = rollerCurrent.getValueAsDouble();
        inputs.armPosition = Units.rotationsToRadians(armPosition.getValueAsDouble());
        inputs.armVelocity = Units.rotationsToRadians(armVelocity.getValueAsDouble());
        inputs.armVoltage = armAppliedVolts.getValueAsDouble();
        inputs.armCurrent = armCurrent.getValueAsDouble();
        inputs.rollerVelocitySetpoint = rollerVelocitySetpoint;
        inputs.armPositionSetpoint = armPositionSetpoint;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        rollerVelocitySetpoint = velocityRadPerSec;
        rollerMotor.setControl(
                rollerVelocityRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void setArmVoltage(double voltage) {
        armMotor.setControl(armVoltageRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setArmPosition(double positionRad) {
        armPositionSetpoint = positionRad;
        armMotor.setControl(armPositionRequest.withPosition(Units.radiansToRotations(positionRad)));
    }

    @Override
    public void stopRoller() {
        rollerVelocitySetpoint = 0.0;
        rollerMotor.stopMotor();
    }

    @Override
    public void stopArmMotor() {
        armMotor.stopMotor();
    }

    @Override
    public void updateRollerConfig() {
        var slot0 = new Slot0Configs();
        applyRollerGains(slot0);
        rollerMotor.getConfigurator().apply(slot0);
    }

    @Override
    public void updateArmConfig() {
        var slot0 = new Slot0Configs();
        applyArmGains(slot0);
        armMotor.getConfigurator().apply(slot0);
        var magic = new MotionMagicConfigs();
        applyArmMotionMagic(magic);
        armMotor.getConfigurator().apply(magic);
    }

    @Override
    public void setArmEncoderPosition(double positionRad) {
        armMotor.setPosition(Units.radiansToRotations(positionRad));
    }

    private static void applyRollerGains(Slot0Configs slot0) {
        slot0.kS = IntakeConstants.ROLLER_KS.get();
        slot0.kV = IntakeConstants.ROLLER_KV.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kA = IntakeConstants.ROLLER_KA.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kP = IntakeConstants.ROLLER_KP.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kI = IntakeConstants.ROLLER_KI.get() * IntakeConstants.ROLLER_GEAR_RATIO;
        slot0.kD = IntakeConstants.ROLLER_KD.get() * IntakeConstants.ROLLER_GEAR_RATIO;
    }

    private static void applyArmGains(Slot0Configs slot0) {
        slot0.kS = IntakeConstants.ARM_KS.get();
        slot0.kV = IntakeConstants.ARM_KV.get() * IntakeConstants.ARM_GEAR_RATIO;
        slot0.kA = IntakeConstants.ARM_KA.get() * IntakeConstants.ARM_GEAR_RATIO;
        slot0.kG = IntakeConstants.ARM_KG.get();
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        slot0.GravityArmPositionOffset = IntakeConstants.ARM_GRAVITY_OFFSET_ROT.get();
        slot0.kP = IntakeConstants.ARM_KP.get() * IntakeConstants.ARM_GEAR_RATIO;
        slot0.kI = IntakeConstants.ARM_KI.get() * IntakeConstants.ARM_GEAR_RATIO;
        slot0.kD = IntakeConstants.ARM_KD.get() * IntakeConstants.ARM_GEAR_RATIO;
    }

    private static void applyArmMotionMagic(MotionMagicConfigs magic) {
        magic.MotionMagicExpo_kV = IntakeConstants.ARM_MOTION_MAGIC_EXPO_KV.get();
        magic.MotionMagicExpo_kA = IntakeConstants.ARM_MOTION_MAGIC_EXPO_KA.get();
        magic.MotionMagicCruiseVelocity =
                Units.radiansToRotations(
                        IntakeConstants.ARM_MOTION_MAGIC_CRUISE_VELOCITY_RAD_PER_SEC.get());
        magic.MotionMagicAcceleration =
                Units.radiansToRotations(
                        IntakeConstants.ARM_MOTION_MAGIC_ACCELERATION_RAD_PER_SEC_SQ.get());
        magic.MotionMagicJerk =
                Units.radiansToRotations(
                        IntakeConstants.ARM_MOTION_MAGIC_JERK_RAD_PER_SEC_CU.get());
    }
}
