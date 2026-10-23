package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import java.util.Optional;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX middleMotor;
    private final TalonFX rightMotor;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VelocityDutyCycle bangBangRequest = new VelocityDutyCycle(0.0).withSlot(1);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);

    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<AngularVelocity> middleVelocity;
    private final StatusSignal<AngularVelocity> rightVelocity;

    private final StatusSignal<Voltage> leftAppliedVolts;
    private final StatusSignal<Voltage> middleAppliedVolts;
    private final StatusSignal<Voltage> rightAppliedVolts;

    private final StatusSignal<Current> leftCurrent;
    private final StatusSignal<Current> middleCurrent;
    private final StatusSignal<Current> rightCurrent;

    private final Optional<Servo> leftHoodServo;
    private final Optional<Servo> rightHoodServo;

    private double velocitySetpoint = 0.0;
    private double hoodServoSetpoint = ShooterConstants.HOOD_SERVO_MIN_POSITION;
    private double hoodSetpointTimestampSeconds = Double.NEGATIVE_INFINITY;

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

        leftHoodServo = createHoodServo(ShooterConstants.LEFT_HOOD_SERVO_PWM_CHANNEL);
        rightHoodServo = createHoodServo(ShooterConstants.RIGHT_HOOD_SERVO_PWM_CHANNEL);
        hoodSetpointTimestampSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        setHoodServoPosition(hoodServoSetpoint);

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
        inputs.requestedVelocity = velocitySetpoint;
        inputs.leftHoodServoPosition =
                leftHoodServo.map(Servo::getPosition).orElse(hoodServoSetpoint);
        inputs.rightHoodServoPosition =
                rightHoodServo.map(Servo::getPosition).orElse(hoodServoSetpoint);
        inputs.hoodServoSetpoint = hoodServoSetpoint;
        inputs.hoodAtSetpoint =
                isHoodHardwareConfigured()
                        && edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
                                        - hoodSetpointTimestampSeconds
                                >= ShooterConstants.HOOD_SERVO_SETTLE_TIME_SECONDS;
        inputs.hoodHardwareConfigured = isHoodHardwareConfigured();
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
    public void runVelocityDutyCycleBangBang(double velocityRadPerSec) {
        velocitySetpoint = velocityRadPerSec;
        double velocityRotationsPerSec = Units.radiansToRotations(velocityRadPerSec);
        leftMotor.setControl(bangBangRequest.withVelocity(velocityRotationsPerSec));
        middleMotor.setControl(bangBangRequest.withVelocity(velocityRotationsPerSec));
        rightMotor.setControl(bangBangRequest.withVelocity(velocityRotationsPerSec));
    }

    @Override
    public void runTorqueCurrent(double torqueCurrentAmps) {
        double output = MathUtil.clamp(torqueCurrentAmps, 0.0, ShooterConstants.SHOOTER_PEAK_TORQUE_CURRENT_AMPS);
        leftMotor.setControl(torqueCurrentRequest.withOutput(output));
        middleMotor.setControl(torqueCurrentRequest.withOutput(output));
        rightMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void setHoodServoPosition(double position) {
        double clampedPosition =
                MathUtil.clamp(
                        position,
                        ShooterConstants.HOOD_SERVO_MIN_POSITION,
                        ShooterConstants.HOOD_SERVO_MAX_POSITION);
        if (Math.abs(clampedPosition - hoodServoSetpoint) > 1.0e-6) {
            hoodSetpointTimestampSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        hoodServoSetpoint = clampedPosition;
        leftHoodServo.ifPresent(servo -> servo.setPosition(hoodServoSetpoint));
        rightHoodServo.ifPresent(servo -> servo.setPosition(hoodServoSetpoint));
    }

    @Override
    public void stop() {
        velocitySetpoint = 0.0;
        setHoodServoPosition(ShooterConstants.HOOD_SERVO_MIN_POSITION);
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

        var slot1 = new Slot1Configs();
        slot1.kP = ShooterConstants.BANG_BANG_KP.get();
        leftMotor.getConfigurator().apply(slot1);
        middleMotor.getConfigurator().apply(slot1);
        rightMotor.getConfigurator().apply(slot1);
    }

    private static void configureMotor(TalonFX motor, InvertedValue inverted) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;
        applyShooterGains(config.Slot0);
        config.Slot1.kP = ShooterConstants.BANG_BANG_KP.get();
        config.TorqueCurrent.PeakForwardTorqueCurrent =
                ShooterConstants.SHOOTER_PEAK_TORQUE_CURRENT_AMPS;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
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

    private static Optional<Servo> createHoodServo(int pwmChannel) {
        if (pwmChannel < 0) {
            return Optional.empty();
        }
        return Optional.of(new Servo(pwmChannel));
    }

    private boolean isHoodHardwareConfigured() {
        return leftHoodServo.isPresent() && rightHoodServo.isPresent();
    }
}
