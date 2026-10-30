package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Three-wheel fixed shooter with a PID-controlled flywheel and servo hood. */
public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double requestedVelocityRadPerSec = 0.0;
    private boolean enabled = false;
    private boolean manualVoltageOverride = false;
    private double commandedVoltage = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (DriverStation.isDisabled()) {
            stop();
        } else if (ShooterConstants.hasAnyShooterConfigChanged()) {
            io.updateShooterConfig();
        }

        if (manualVoltageOverride && !allFlywheelsConnected()) {
            enabled = false;
            requestedVelocityRadPerSec = 0.0;
            manualVoltageOverride = false;
            stopFlywheels();
        }

        if (manualVoltageOverride) {
            inputs.requestedVelocity = 0.0;
            inputs.velocitySetpoint = 0.0;
        } else if (!enabled || requestedVelocityRadPerSec <= 1e-3) {
            stopFlywheels();
            inputs.requestedVelocity = requestedVelocityRadPerSec;
            inputs.velocitySetpoint = requestedVelocityRadPerSec;
        } else if (!allFlywheelsConnected()) {
            stopFlywheels();
            inputs.requestedVelocity = requestedVelocityRadPerSec;
            inputs.velocitySetpoint = requestedVelocityRadPerSec;
        } else {
            commandedVoltage = 0.0;
            io.setVelocity(requestedVelocityRadPerSec);
            inputs.requestedVelocity = requestedVelocityRadPerSec;
            inputs.velocitySetpoint = requestedVelocityRadPerSec;
        }

        Logger.processInputs("Shooter", inputs);
        logTelemetry();
    }

    private boolean allFlywheelsConnected() {
        return inputs.leftMotorConnected
                && inputs.middleMotorConnected
                && inputs.rightMotorConnected;
    }

    private void stopFlywheels() {
        io.stop();
        commandedVoltage = 0.0;
    }

    private static double rpmToRadPerSec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    private double velocityToleranceRadPerSec() {
        return rpmToRadPerSec(ShooterConstants.getVelocityToleranceRPM());
    }

    private double averageFlywheelVelocity() {
        return (inputs.leftVelocity + inputs.middleVelocity + inputs.rightVelocity) / 3.0;
    }

    private void logTelemetry() {
        Logger.recordOutput("Shooter/LeftPositionRad", inputs.leftPosition);
        Logger.recordOutput("Shooter/MiddlePositionRad", inputs.middlePosition);
        Logger.recordOutput("Shooter/RightPositionRad", inputs.rightPosition);
        Logger.recordOutput("Shooter/LeftVelocityRadPerSec", inputs.leftVelocity);
        Logger.recordOutput("Shooter/MiddleVelocityRadPerSec", inputs.middleVelocity);
        Logger.recordOutput("Shooter/RightVelocityRadPerSec", inputs.rightVelocity);
        Logger.recordOutput("Shooter/VelocitySetpointRadPerSec", requestedVelocityRadPerSec);
        Logger.recordOutput("Shooter/VelocitySetpointRPM", getVelocitySetpointRPM());
        Logger.recordOutput(
                "Shooter/AverageVelocityRPM",
                Units.radiansPerSecondToRotationsPerMinute(averageFlywheelVelocity()));
        Logger.recordOutput(
                "Shooter/MotionMagic/AccelerationRotationsPerSecSq",
                ShooterConstants.getMotionMagicAccelerationRotationsPerSecSq());
        Logger.recordOutput(
                "Shooter/VelocityToleranceRPM", ShooterConstants.getVelocityToleranceRPM());
        Logger.recordOutput("Shooter/LeftVelocityRPM", getLeftVelocityRPM());
        Logger.recordOutput("Shooter/MiddleVelocityRPM", getMiddleVelocityRPM());
        Logger.recordOutput("Shooter/RightVelocityRPM", getRightVelocityRPM());
        Logger.recordOutput("Shooter/LeftVoltage", inputs.leftVoltage);
        Logger.recordOutput("Shooter/MiddleVoltage", inputs.middleVoltage);
        Logger.recordOutput("Shooter/RightVoltage", inputs.rightVoltage);
        Logger.recordOutput("Shooter/LeftCurrentAmps", inputs.leftCurrent);
        Logger.recordOutput("Shooter/MiddleCurrentAmps", inputs.middleCurrent);
        Logger.recordOutput("Shooter/RightCurrentAmps", inputs.rightCurrent);
        Logger.recordOutput("Shooter/CommandedVoltage", commandedVoltage);
        Logger.recordOutput("Shooter/LeftMotorConnected", inputs.leftMotorConnected);
        Logger.recordOutput("Shooter/MiddleMotorConnected", inputs.middleMotorConnected);
        Logger.recordOutput("Shooter/RightMotorConnected", inputs.rightMotorConnected);
        Logger.recordOutput("Shooter/MotorSignalFault", !allFlywheelsConnected());
        Logger.recordOutput(
                "Shooter/UnlicensedFeatureFault/Left", inputs.leftUnlicensedFeatureFault);
        Logger.recordOutput(
                "Shooter/UnlicensedFeatureFault/Middle", inputs.middleUnlicensedFeatureFault);
        Logger.recordOutput(
                "Shooter/UnlicensedFeatureFault/Right", inputs.rightUnlicensedFeatureFault);
        Logger.recordOutput(
                "Shooter/UnlicensedFeatureFault/Any",
                inputs.leftUnlicensedFeatureFault
                        || inputs.middleUnlicensedFeatureFault
                        || inputs.rightUnlicensedFeatureFault);
        Logger.recordOutput(
                "Shooter/LeftErrorRPM",
                Units.radiansPerSecondToRotationsPerMinute(
                        requestedVelocityRadPerSec - inputs.leftVelocity));
        Logger.recordOutput(
                "Shooter/MiddleErrorRPM",
                Units.radiansPerSecondToRotationsPerMinute(
                        requestedVelocityRadPerSec - inputs.middleVelocity));
        Logger.recordOutput(
                "Shooter/RightErrorRPM",
                Units.radiansPerSecondToRotationsPerMinute(
                        requestedVelocityRadPerSec - inputs.rightVelocity));
        Logger.recordOutput("Shooter/Hood/LeftServoPosition", inputs.leftHoodServoPosition);
        Logger.recordOutput("Shooter/Hood/RightServoPosition", inputs.rightHoodServoPosition);
        Logger.recordOutput("Shooter/Hood/ServoSetpoint", inputs.hoodServoSetpoint);
        Logger.recordOutput(
                "Shooter/Hood/RequestedAngleRad",
                ShooterConstants.servoPositionToHoodAngle(inputs.hoodServoSetpoint));
        Logger.recordOutput(
                "Shooter/Hood/RequestedAngleDeg",
                Math.toDegrees(
                        ShooterConstants.servoPositionToHoodAngle(inputs.hoodServoSetpoint)));
        Logger.recordOutput("Shooter/HoodAtSetpoint", hoodAtSetpoint());
        Logger.recordOutput("Shooter/HoodHardwareConfigured", inputs.hoodHardwareConfigured);
        Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
        Logger.recordOutput("Shooter/IsRunning", isRunning());
        Logger.recordOutput("Shooter/FlywheelAtSetpoint", flywheelsAtSetpoint());
        for (ShooterConstants.Flywheel flywheel : ShooterConstants.Flywheel.values()) {
            flywheel.kS.get();
            flywheel.kV.get();
            flywheel.kA.get();
            flywheel.kP.get();
            flywheel.kI.get();
            flywheel.kD.get();
        }
        ShooterConstants.MAX_VELOCITY_RPM.get();
        ShooterConstants.TUNING_VELOCITY_RPM.get();
        ShooterConstants.TUNING_HOOD_ANGLE_DEG.get();
        ShooterConstants.CHARACTERIZATION_RAMP_VOLTS_PER_SEC.get();
        ShooterConstants.CHARACTERIZATION_MAX_VOLTS.get();
        ShooterConstants.SHOOTER_STATOR_CURRENT_LIMIT_AMPS.get();
        ShooterConstants.SHOOTER_PEAK_TORQUE_CURRENT_AMPS.get();
        ShooterConstants.MOTION_MAGIC_ACCELERATION_ROTATIONS_PER_SEC_SQ.get();
    }

    public void setVelocity(double velocityRadPerSec) {
        if (!Double.isFinite(velocityRadPerSec)) {
            velocityRadPerSec = 0.0;
        }
        requestedVelocityRadPerSec =
                Math.max(
                        0.0,
                        Math.min(
                                velocityRadPerSec,
                                rpmToRadPerSec(ShooterConstants.getMaxVelocityRPM())));
        enabled = requestedVelocityRadPerSec > 1e-3;
        manualVoltageOverride = false;
        if (!enabled) {
            stop();
        }
    }

    public void setHoodAngle(double angleRad) {
        io.setHoodServoPosition(ShooterConstants.hoodAngleToServoPosition(angleRad));
    }

    /** Applies open-loop voltage to all three flywheels for feedforward characterization. */
    public void runVoltage(double volts) {
        if (!allFlywheelsConnected()) {
            enabled = false;
            requestedVelocityRadPerSec = 0.0;
            manualVoltageOverride = false;
            stopFlywheels();
            return;
        }
        if (!Double.isFinite(volts)) {
            volts = 0.0;
        }
        double maxVolts = ShooterConstants.getCharacterizationMaxVolts();
        double clampedVolts = Math.max(-maxVolts, Math.min(maxVolts, volts));
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = true;
        commandedVoltage = clampedVolts;
        io.runVoltage(clampedVolts);
    }

    public void stop() {
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = false;
        stopFlywheels();
        setHoodAngle(ShooterConstants.HOOD_MIN_ANGLE_RAD);
    }

    public double getLeftVelocityRadPerSec() {
        return inputs.leftVelocity;
    }

    public double getMiddleVelocityRadPerSec() {
        return inputs.middleVelocity;
    }

    public double getRightVelocityRadPerSec() {
        return inputs.rightVelocity;
    }

    public double getLeftAppliedVoltage() {
        return inputs.leftVoltage;
    }

    public double getMiddleAppliedVoltage() {
        return inputs.middleVoltage;
    }

    public double getRightAppliedVoltage() {
        return inputs.rightVoltage;
    }

    public double getLeftVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.leftVelocity);
    }

    public double getMiddleVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.middleVelocity);
    }

    public double getRightVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.rightVelocity);
    }

    public double getVelocitySetpointRadPerSec() {
        return requestedVelocityRadPerSec;
    }

    public double getVelocitySetpointRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(requestedVelocityRadPerSec);
    }

    public double getHoodPositionRad() {
        // PWM servos do not report measured mechanism position; this is the commanded angle.
        return ShooterConstants.servoPositionToHoodAngle(inputs.hoodServoSetpoint);
    }

    public double getHoodSetpointRad() {
        return getHoodPositionRad();
    }

    public boolean atSetpoint() {
        return flywheelsAtSetpoint() && hoodAtSetpoint();
    }

    public boolean flywheelsAtSetpoint() {
        double tolerance = velocityToleranceRadPerSec();
        if (!allFlywheelsConnected()
                || !Double.isFinite(tolerance)
                || requestedVelocityRadPerSec <= tolerance) {
            return false;
        }
        return Math.abs(requestedVelocityRadPerSec - inputs.leftVelocity) < tolerance
                && Math.abs(requestedVelocityRadPerSec - inputs.middleVelocity) < tolerance
                && Math.abs(requestedVelocityRadPerSec - inputs.rightVelocity) < tolerance;
    }

    public boolean hoodAtSetpoint() {
        return inputs.hoodAtSetpoint;
    }

    public boolean isRunning() {
        double tolerance = velocityToleranceRadPerSec();
        return Math.abs(inputs.leftVelocity) > tolerance
                || Math.abs(inputs.middleVelocity) > tolerance
                || Math.abs(inputs.rightVelocity) > tolerance;
    }
}
