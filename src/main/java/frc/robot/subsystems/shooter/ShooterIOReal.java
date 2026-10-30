package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.Servo;
import java.util.Optional;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX middleMotor;
    private final TalonFX rightMotor;

    private final MotionMagicVelocityTorqueCurrentFOC velocityRequest =
            new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> leftVelocity;
    private final StatusSignal<Angle> leftPosition;
    private final StatusSignal<AngularVelocity> middleVelocity;
    private final StatusSignal<Angle> middlePosition;
    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<Boolean> leftUnlicensedFeatureFault;
    private final StatusSignal<Boolean> middleUnlicensedFeatureFault;
    private final StatusSignal<Boolean> rightUnlicensedFeatureFault;

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

        configureMotor(
                leftMotor, InvertedValue.CounterClockwise_Positive, ShooterConstants.Flywheel.LEFT);
        configureMotor(
                middleMotor,
                InvertedValue.CounterClockwise_Positive,
                ShooterConstants.Flywheel.MIDDLE);
        configureMotor(
                rightMotor, InvertedValue.Clockwise_Positive, ShooterConstants.Flywheel.RIGHT);

        leftVelocity = leftMotor.getVelocity();
        leftPosition = leftMotor.getPosition();
        middleVelocity = middleMotor.getVelocity();
        middlePosition = middleMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightPosition = rightMotor.getPosition();
        leftUnlicensedFeatureFault = leftMotor.getFault_UnlicensedFeatureInUse();
        middleUnlicensedFeatureFault = middleMotor.getFault_UnlicensedFeatureInUse();
        rightUnlicensedFeatureFault = rightMotor.getFault_UnlicensedFeatureInUse();

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
                leftPosition,
                middleVelocity,
                middlePosition,
                rightVelocity,
                rightPosition,
                leftUnlicensedFeatureFault,
                middleUnlicensedFeatureFault,
                rightUnlicensedFeatureFault,
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
                leftPosition,
                middleVelocity,
                middlePosition,
                rightVelocity,
                rightPosition,
                leftUnlicensedFeatureFault,
                middleUnlicensedFeatureFault,
                rightUnlicensedFeatureFault,
                leftAppliedVolts,
                middleAppliedVolts,
                rightAppliedVolts,
                leftCurrent,
                middleCurrent,
                rightCurrent);

        inputs.leftPosition = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.middlePosition = Units.rotationsToRadians(middlePosition.getValueAsDouble());
        inputs.rightPosition = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.leftVelocity = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
        inputs.middleVelocity = Units.rotationsToRadians(middleVelocity.getValueAsDouble());
        inputs.rightVelocity = Units.rotationsToRadians(rightVelocity.getValueAsDouble());

        inputs.leftVoltage = leftAppliedVolts.getValueAsDouble();
        inputs.middleVoltage = middleAppliedVolts.getValueAsDouble();
        inputs.rightVoltage = rightAppliedVolts.getValueAsDouble();

        inputs.leftCurrent = leftCurrent.getValueAsDouble();
        inputs.middleCurrent = middleCurrent.getValueAsDouble();
        inputs.rightCurrent = rightCurrent.getValueAsDouble();
        inputs.leftMotorConnected = leftVelocity.getStatus().isOK();
        inputs.middleMotorConnected = middleVelocity.getStatus().isOK();
        inputs.rightMotorConnected = rightVelocity.getStatus().isOK();
        inputs.leftUnlicensedFeatureFault = leftUnlicensedFeatureFault.getValue();
        inputs.middleUnlicensedFeatureFault = middleUnlicensedFeatureFault.getValue();
        inputs.rightUnlicensedFeatureFault = rightUnlicensedFeatureFault.getValue();

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
        velocityRequest.withAcceleration(0.0).withVelocity(velocityRotationsPerSec);
        leftMotor.setControl(velocityRequest);
        middleMotor.setControl(velocityRequest);
        rightMotor.setControl(velocityRequest);
    }

    @Override
    public void runVoltage(double volts) {
        leftMotor.setControl(voltageRequest.withOutput(volts));
        middleMotor.setControl(voltageRequest.withOutput(volts));
        rightMotor.setControl(voltageRequest.withOutput(volts));
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
        leftMotor.stopMotor();
        middleMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void updateShooterConfig() {
        updateMotorConfig(leftMotor, ShooterConstants.Flywheel.LEFT);
        updateMotorConfig(middleMotor, ShooterConstants.Flywheel.MIDDLE);
        updateMotorConfig(rightMotor, ShooterConstants.Flywheel.RIGHT);
    }

    private static void configureMotor(
            TalonFX motor, InvertedValue inverted, ShooterConstants.Flywheel flywheel) {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.getStatorCurrentLimitAmps();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOTER_GEAR_RATIO;
        applyShooterGains(config.Slot0, flywheel);
        config.MotionMagic.MotionMagicAcceleration =
                ShooterConstants.getMotionMagicAccelerationRotationsPerSecSq();
        config.MotionMagic.MotionMagicJerk = 0.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.getPeakTorqueCurrentAmps();
        config.TorqueCurrent.PeakReverseTorqueCurrent =
                -ShooterConstants.getPeakTorqueCurrentAmps();
        motor.getConfigurator().apply(config);
    }

    private static void updateMotorConfig(TalonFX motor, ShooterConstants.Flywheel flywheel) {
        var slot0 = new Slot0Configs();
        applyShooterGains(slot0, flywheel);
        motor.getConfigurator().apply(slot0);

        var motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration =
                ShooterConstants.getMotionMagicAccelerationRotationsPerSecSq();
        motionMagic.MotionMagicJerk = 0.0;
        motor.getConfigurator().apply(motionMagic);

        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = ShooterConstants.getStatorCurrentLimitAmps();
        currentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(currentLimits);

        var torqueCurrent = new com.ctre.phoenix6.configs.TorqueCurrentConfigs();
        torqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.getPeakTorqueCurrentAmps();
        torqueCurrent.PeakReverseTorqueCurrent = -ShooterConstants.getPeakTorqueCurrentAmps();
        motor.getConfigurator().apply(torqueCurrent);
    }

    private static void applyShooterGains(Slot0Configs slot0, ShooterConstants.Flywheel flywheel) {
        // Motion Magic Velocity TorqueCurrentFOC gains produce amps. SensorToMechanismRatio
        // already makes velocity/error mechanism-side, so do not scale gains by gear ratio.
        slot0.kS = flywheel.kS.getClamped(0.0, 12.0);
        slot0.kV = flywheel.kV.getClamped(0.0, 12.0);
        slot0.kA = flywheel.kA.getClamped(0.0, 12.0);
        slot0.kP = flywheel.kP.getClamped(0.0, 100.0);
        slot0.kI = flywheel.kI.getClamped(0.0, 100.0);
        slot0.kD = flywheel.kD.getClamped(0.0, 100.0);
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
