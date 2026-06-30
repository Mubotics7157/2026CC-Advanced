package frc.robot.subsystems.shooter;

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

public class ShooterIOReal implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX middleMotor;
    private final TalonFX rightMotor;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);

    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<AngularVelocity> middleVelocity;
    private final StatusSignal<AngularVelocity> rightVelocity;

    private final StatusSignal<Voltage> leftAppliedVolts;
    private final StatusSignal<Voltage> middleAppliedVolts;
    private final StatusSignal<Voltage> rightAppliedVolts;

    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Current> middleCurrent;
    private final StatusSignal<Current> rightCurrent;

    private double velocitySetpoint = 0.0;

    public ShooterIOReal() {
        leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CAN_BUS);
        middleMotor = new TalonFX(ShooterConstants.MIDDLE_MOTOR_ID, ShooterConstants.CAN_BUS);
        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CAN_BUS);

        configureMotor(leftMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(middleMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(rightMotor, InvertedValue.Clockwise_Positive);

        leftVelocity = leftMotor.getVelocity();
        middleVelocity = middleMotor.getVelocity();
        rightVelocity = rightMotor.getVelocity();

        leftAppliedVolts = leftMotor.getMotorVoltage();
        middleAppliedVolts = middleMotor.getMotorVoltage();
        rightAppliedVolts = rightMotor.getMotorVoltage();

        leftCurrent = leftMotor.getStatorCurrent();
        middleCurrent = middleMotor.getStatorCurrent();
        rightCurrent = rightMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ShooterConstants.STATUS_SIGNAL_UPDATE_FREQUENCY,
                leftVelocity,
                middleVelocity,
                rightVelocity,
                leftAppliedVolts,
                middleAppliedVolts,
                rightAppliedVolts,
                leftCurrent,
                middleCurrent,
                rightCurrent);

        leftMotor.optimizeBusUtilization();
        middleMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                middleVelocity,
                rightVelocity,
                leftAppliedVolts,
                middleAppliedVolts,
                rightAppliedVolts,
                leftCurrent,
                middleCurrent,
                rightCurrent);

        inputs.leftVelocity = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
        inputs.middleVelocity = Units.rotationsToRadians(middleVelocity.getValueAsDouble());
        inputs.rightVelocity = Units.rotationsToRadians(rightVelocity.getValueAsDouble());

        inputs.leftVoltage = leftAppliedVolts.getValueAsDouble();
        inputs.middleVoltage = middleAppliedVolts.getValueAsDouble();
        inputs.rightVoltage = rightAppliedVolts.getValueAsDouble();

        inputs.leftCurrent = leftCurrent.getValueAsDouble();
        inputs.middleCurrent = middleCurrent.getValueAsDouble();
        inputs.rightCurrent = rightCurrent.getValueAsDouble();

        inputs.velocitySetpoint = velocitySetpoint;
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        velocitySetpoint = velocityRadPerSec;
        double velocityRotationsPerSec = Units.radiansToRotations(velocityRadPerSec);
        leftMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
        middleMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
        rightMotor.setControl(velocityRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void stop() {
        velocitySetpoint = 0.0;
        leftMotor.stopMotor();
        middleMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void updateShooterConfig() {
        var slot0 = new Slot0Configs();
        applyShooterGains(slot0);
        leftMotor.getConfigurator().apply(slot0);
        middleMotor.getConfigurator().apply(slot0);
        rightMotor.getConfigurator().apply(slot0);
    }

    private static void configureMotor(TalonFX motor, InvertedValue inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;
        applyShooterGains(config.Slot0);
        motor.getConfigurator().apply(config);
    }

    private static void applyShooterGains(Slot0Configs slot0) {
        slot0.kS = ShooterConstants.SHOOTER_KS.get();
        slot0.kV = ShooterConstants.SHOOTER_KV.get() * ShooterConstants.SHOOTER_GEAR_RATIO;
        slot0.kA = ShooterConstants.SHOOTER_KA.get() * ShooterConstants.SHOOTER_GEAR_RATIO;
        slot0.kP = ShooterConstants.SHOOTER_KP.get() * ShooterConstants.SHOOTER_GEAR_RATIO;
        slot0.kI = ShooterConstants.SHOOTER_KI.get() * ShooterConstants.SHOOTER_GEAR_RATIO;
        slot0.kD = ShooterConstants.SHOOTER_KD.get() * ShooterConstants.SHOOTER_GEAR_RATIO;
    }
}
