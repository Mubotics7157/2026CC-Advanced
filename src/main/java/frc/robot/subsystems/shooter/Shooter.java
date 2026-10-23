package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Three-wheel fixed shooter with startup, idle, ball-contact, and recovery control phases. */
public class Shooter extends SubsystemBase {
    public enum ControlPhase {
        DISABLED,
        STARTUP,
        IDLE,
        BALL,
        RECOVERY,
        MANUAL_VOLTAGE
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double requestedVelocityRadPerSec = 0.0;
    private boolean enabled = false;
    private boolean launchActive = false;
    private boolean manualVoltageOverride = false;
    private boolean previousLaunchActive = false;
    private boolean ballEventArmed = true;
    private boolean ballEntryDetectedThisCycle = false;
    private double phaseStartTimestampSec = 0.0;
    private ControlPhase phase = ControlPhase.DISABLED;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        ballEntryDetectedThisCycle = false;

        if (ShooterConstants.hasAnyShooterPIDChanged()) {
            io.updateShooterConfig();
        }

        if (manualVoltageOverride) {
            inputs.requestedVelocity = 0.0;
            inputs.velocitySetpoint = 0.0;
            inputs.controlPhase = phase.name();
            Logger.processInputs("Shooter", inputs);
            logTelemetry();
            previousLaunchActive = launchActive;
            return;
        }

        if (!launchActive) {
            ballEventArmed = true;
        }

        if (!enabled || requestedVelocityRadPerSec <= 1e-3) {
            io.stop();
            phase = ControlPhase.DISABLED;
        } else {
            runPhaseController();
        }

        inputs.requestedVelocity = requestedVelocityRadPerSec;
        inputs.velocitySetpoint = requestedVelocityRadPerSec;
        inputs.controlPhase = phase.name();
        Logger.processInputs("Shooter", inputs);
        logTelemetry();
        previousLaunchActive = launchActive;
    }

    private void runPhaseController() {
        switch (phase) {
            case DISABLED -> {
                transitionTo(ControlPhase.STARTUP);
                io.runVelocityDutyCycleBangBang(requestedVelocityRadPerSec);
            }
            case STARTUP -> {
                io.runVelocityDutyCycleBangBang(requestedVelocityRadPerSec);
                if (isNearTarget(rpmToRadPerSec(ShooterConstants.STARTUP_READY_RPM.get()))) {
                    transitionTo(ControlPhase.IDLE);
                }
            }
            case IDLE -> {
                io.setVelocity(requestedVelocityRadPerSec);
                boolean launchRisingEdge = launchActive && !previousLaunchActive;
                boolean ballEntryByVelocityDrop =
                        launchActive
                                && requestedVelocityRadPerSec - averageFlywheelVelocity()
                                        > rpmToRadPerSec(
                                                Math.max(
                                                        0.0,
                                                        ShooterConstants.BALL_ENTRY_DROP_RPM
                                                                .get()));
                if (ballEventArmed && (launchRisingEdge || ballEntryByVelocityDrop)) {
                    ballEventArmed = false;
                    ballEntryDetectedThisCycle = true;
                    transitionTo(ControlPhase.BALL);
                }
            }
            case BALL -> {
                io.runTorqueCurrent(ShooterConstants.BALL_TORQUE_CURRENT_AMPS.get());
                if (Timer.getFPGATimestamp() - phaseStartTimestampSec
                        >= ShooterConstants.BALL_PHASE_DURATION_SEC.get()) {
                    transitionTo(ControlPhase.RECOVERY);
                }
            }
            case RECOVERY -> {
                io.runVelocityDutyCycleBangBang(requestedVelocityRadPerSec);
                if (isNearTarget(rpmToRadPerSec(ShooterConstants.RECOVERY_READY_RPM.get()))) {
                    ballEventArmed = true;
                    transitionTo(ControlPhase.IDLE);
                }
            }
        }
    }

    private void transitionTo(ControlPhase newPhase) {
        phase = newPhase;
        phaseStartTimestampSec = Timer.getFPGATimestamp();
    }

    private boolean isNearTarget(double toleranceRadPerSec) {
        return Math.abs(requestedVelocityRadPerSec - averageFlywheelVelocity())
                < toleranceRadPerSec;
    }

    private static double rpmToRadPerSec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    private double velocityToleranceRadPerSec() {
        return rpmToRadPerSec(Math.max(0.0, ShooterConstants.VELOCITY_TOLERANCE_RPM.get()));
    }

    private double averageFlywheelVelocity() {
        return (inputs.leftVelocity + inputs.middleVelocity + inputs.rightVelocity) / 3.0;
    }

    private void logTelemetry() {
        Logger.recordOutput("Shooter/LeftVelocityRadPerSec", inputs.leftVelocity);
        Logger.recordOutput("Shooter/MiddleVelocityRadPerSec", inputs.middleVelocity);
        Logger.recordOutput("Shooter/RightVelocityRadPerSec", inputs.rightVelocity);
        Logger.recordOutput("Shooter/VelocitySetpointRadPerSec", requestedVelocityRadPerSec);
        Logger.recordOutput("Shooter/ControlPhase", phase.name());
        Logger.recordOutput("Shooter/LaunchActive", launchActive);
        Logger.recordOutput("Shooter/BallEntryDetected", ballEntryDetectedThisCycle);
        Logger.recordOutput("Shooter/LeftVoltage", inputs.leftVoltage);
        Logger.recordOutput("Shooter/MiddleVoltage", inputs.middleVoltage);
        Logger.recordOutput("Shooter/RightVoltage", inputs.rightVoltage);
        Logger.recordOutput("Shooter/LeftCurrentAmps", inputs.leftCurrent);
        Logger.recordOutput("Shooter/MiddleCurrentAmps", inputs.middleCurrent);
        Logger.recordOutput("Shooter/RightCurrentAmps", inputs.rightCurrent);
        Logger.recordOutput("Shooter/LeftMotorConnected", inputs.leftMotorConnected);
        Logger.recordOutput("Shooter/MiddleMotorConnected", inputs.middleMotorConnected);
        Logger.recordOutput("Shooter/RightMotorConnected", inputs.rightMotorConnected);
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
    }

    public void setVelocity(double velocityRadPerSec) {
        if (manualVoltageOverride) {
            manualVoltageOverride = false;
            phase = ControlPhase.DISABLED;
        }
        requestedVelocityRadPerSec =
                Math.max(
                        0.0,
                        Math.min(
                                velocityRadPerSec,
                                rpmToRadPerSec(
                                        Math.min(
                                                6000.0,
                                                Math.max(
                                                        0.0,
                                                        ShooterConstants.MAX_VELOCITY_RPM
                                                                .get())))));
        enabled = requestedVelocityRadPerSec > 1e-3;
        manualVoltageOverride = false;
        if (!enabled) {
            stop();
        }
    }

    public void setHoodAngle(double angleRad) {
        io.setHoodServoPosition(ShooterConstants.hoodAngleToServoPosition(angleRad));
    }

    public void setLaunchActive(boolean active) {
        launchActive = active;
        if (!active) {
            ballEventArmed = true;
        }
    }

    /** Applies open-loop voltage to all three flywheels until a velocity request or stop occurs. */
    public void runVoltage(double volts) {
        double maxVolts =
                Math.min(6.0, Math.max(0.0, ShooterConstants.CHARACTERIZATION_MAX_VOLTS.get()));
        double clampedVolts = Math.max(-maxVolts, Math.min(maxVolts, volts));
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = true;
        phase = ControlPhase.MANUAL_VOLTAGE;
        io.runVoltage(clampedVolts);
    }

    public void stop() {
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = false;
        launchActive = false;
        ballEventArmed = true;
        phase = ControlPhase.DISABLED;
        io.stop();
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

    public ControlPhase getControlPhase() {
        return phase;
    }

    public double getVelocityRecoveryRatio() {
        if (requestedVelocityRadPerSec <= 1e-3) {
            return 1.0;
        }
        return Math.max(0.0, Math.min(1.0, averageFlywheelVelocity() / requestedVelocityRadPerSec));
    }

    public double getHoodPositionRad() {
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
        if (requestedVelocityRadPerSec <= tolerance) {
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
