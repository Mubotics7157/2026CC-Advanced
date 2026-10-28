package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Three-wheel fixed shooter with startup, idle, ball-contact, and recovery control phases. */
public class Shooter extends SubsystemBase {
    private enum ControlOutput {
        STOP,
        VELOCITY,
        BANG_BANG,
        TORQUE_CURRENT,
        MANUAL_VOLTAGE
    }

    public enum ControlPhase {
        DISABLED,
        STARTUP,
        IDLE,
        BALL_PENDING,
        BALL,
        RECOVERY,
        MANUAL_VOLTAGE,
        FAULT
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
    private boolean launchRisingEdgeThisCycle = false;
    private double phaseStartTimestampSec = 0.0;
    private double ballVelocityReferenceRPM = 0.0;
    private ControlPhase phase = ControlPhase.DISABLED;
    private ControlOutput controlOutput = ControlOutput.STOP;
    private double commandedTorqueCurrentAmps = 0.0;
    private double commandedVoltage = 0.0;
    private String ballEntrySource = "NONE";
    private String lastPhaseTransition = "NONE";
    private double lastPhaseDurationSec = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        ballEntryDetectedThisCycle = false;
        launchRisingEdgeThisCycle = launchActive && !previousLaunchActive;
        ballEntrySource = "NONE";
        if (phase == ControlPhase.IDLE && (!launchActive || launchRisingEdgeThisCycle)) {
            ballVelocityReferenceRPM = getAverageVelocityRPM();
        }

        if (DriverStation.isDisabled()) {
            stop();
        } else if (ShooterConstants.hasAnyShooterPIDChanged()) {
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

        if (!launchActive && phase == ControlPhase.IDLE) {
            ballEventArmed = true;
        }

        if (!enabled || requestedVelocityRadPerSec <= 1e-3) {
            stopFlywheels();
            setPhase(ControlPhase.DISABLED, "NO_VELOCITY_REQUEST");
        } else {
            runPhaseController(Timer.getFPGATimestamp());
        }

        inputs.requestedVelocity = requestedVelocityRadPerSec;
        inputs.velocitySetpoint = requestedVelocityRadPerSec;
        inputs.controlPhase = phase.name();
        Logger.processInputs("Shooter", inputs);
        logTelemetry();
        previousLaunchActive = launchActive;
    }

    private void runPhaseController(double nowSec) {
        if (phase != ControlPhase.FAULT && !allFlywheelsConnected()) {
            setPhase(ControlPhase.FAULT, "MOTOR_SIGNAL_DISCONNECTED");
        } else if (phase == ControlPhase.FAULT && allFlywheelsConnected()) {
            setPhase(ControlPhase.STARTUP, "MOTOR_SIGNALS_RESTORED");
        }

        switch (phase) {
            case DISABLED -> setPhase(ControlPhase.STARTUP, "VELOCITY_REQUESTED");
            case STARTUP -> {
                double readyToleranceRPM =
                        ShooterConstants.getStartupReadyToleranceRPM(getVelocitySetpointRPM());
                if (isNearTarget(readyToleranceRPM)) {
                    setPhase(ControlPhase.IDLE, "STARTUP_READY");
                }
            }
            case IDLE -> {
                if (ballEventArmed && launchActive) {
                    if (isBallVelocityDropDetected()) {
                        ballEventArmed = false;
                        enterBallPhase("VELOCITY_DROP");
                    } else if (launchRisingEdgeThisCycle) {
                        ballEventArmed = false;
                        if (ShooterConstants.getLaunchToBallDelaySeconds() <= 0.0) {
                            enterBallPhase("LAUNCH_RISING_EDGE");
                        } else {
                            setPhase(ControlPhase.BALL_PENDING, "LAUNCH_RISING_EDGE");
                        }
                    }
                }
            }
            case BALL_PENDING -> {
                if (!launchActive) {
                    ballEventArmed = true;
                    setPhase(ControlPhase.IDLE, "FEED_CANCELLED");
                } else if (isBallVelocityDropDetected()) {
                    enterBallPhase("VELOCITY_DROP");
                } else if (nowSec - phaseStartTimestampSec
                        >= ShooterConstants.getLaunchToBallDelaySeconds()) {
                    enterBallPhase("LAUNCH_DELAY_ELAPSED");
                }
            }
            case BALL -> {
                if (nowSec - phaseStartTimestampSec
                        >= ShooterConstants.getBallPhaseDurationSeconds()) {
                    setPhase(ControlPhase.RECOVERY, "BALL_PULSE_COMPLETE");
                }
            }
            case RECOVERY -> {
                double readyToleranceRPM =
                        ShooterConstants.getRecoveryReadyToleranceRPM(getVelocitySetpointRPM());
                if (isNearTarget(readyToleranceRPM)) {
                    ballEventArmed = true;
                    setPhase(ControlPhase.IDLE, "RECOVERY_READY");
                }
            }
            case MANUAL_VOLTAGE -> setPhase(ControlPhase.STARTUP, "VELOCITY_REQUESTED");
            case FAULT -> {
                // Hold outputs off until all three motor velocity signals recover.
            }
        }
        applyPhaseOutput();
    }

    private void enterBallPhase(String reason) {
        ballEntryDetectedThisCycle = true;
        ballEntrySource = reason;
        setPhase(ControlPhase.BALL, reason);
    }

    private boolean isBallVelocityDropDetected() {
        double measuredDropRPM = ballVelocityReferenceRPM - getAverageVelocityRPM();
        double belowTargetRPM = getVelocitySetpointRPM() - getAverageVelocityRPM();
        return Math.min(measuredDropRPM, belowTargetRPM) > ShooterConstants.getBallEntryDropRPM();
    }

    private double getAverageVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(averageFlywheelVelocity());
    }

    private void setPhase(ControlPhase newPhase, String reason) {
        if (phase == newPhase) {
            return;
        }
        double nowSec = Timer.getFPGATimestamp();
        if (phase != ControlPhase.DISABLED && phase != ControlPhase.MANUAL_VOLTAGE) {
            lastPhaseDurationSec = Math.max(0.0, nowSec - phaseStartTimestampSec);
        }
        lastPhaseTransition = phase.name() + "->" + newPhase.name() + ":" + reason;
        phase = newPhase;
        phaseStartTimestampSec = nowSec;
        if (newPhase == ControlPhase.IDLE) {
            ballVelocityReferenceRPM = getAverageVelocityRPM();
        }
    }

    private void applyPhaseOutput() {
        commandedTorqueCurrentAmps = 0.0;
        commandedVoltage = 0.0;
        switch (phase) {
            case STARTUP, RECOVERY -> {
                controlOutput = ControlOutput.BANG_BANG;
                io.runVelocityDutyCycleBangBang(requestedVelocityRadPerSec);
            }
            case IDLE, BALL_PENDING -> {
                controlOutput = ControlOutput.VELOCITY;
                io.setVelocity(requestedVelocityRadPerSec);
            }
            case BALL -> {
                controlOutput = ControlOutput.TORQUE_CURRENT;
                commandedTorqueCurrentAmps = ShooterConstants.getBallTorqueCurrentAmps();
                io.runTorqueCurrent(commandedTorqueCurrentAmps);
            }
            case DISABLED, MANUAL_VOLTAGE, FAULT -> stopFlywheels();
        }
    }

    private boolean isNearTarget(double toleranceRPM) {
        if (!allFlywheelsConnected()) {
            return false;
        }
        double toleranceRadPerSec = rpmToRadPerSec(toleranceRPM);
        return Double.isFinite(toleranceRadPerSec)
                && Math.abs(requestedVelocityRadPerSec - inputs.leftVelocity) < toleranceRadPerSec
                && Math.abs(requestedVelocityRadPerSec - inputs.middleVelocity) < toleranceRadPerSec
                && Math.abs(requestedVelocityRadPerSec - inputs.rightVelocity) < toleranceRadPerSec;
    }

    private boolean allFlywheelsConnected() {
        return inputs.leftMotorConnected
                && inputs.middleMotorConnected
                && inputs.rightMotorConnected;
    }

    private void stopFlywheels() {
        io.stop();
        controlOutput = ControlOutput.STOP;
        commandedTorqueCurrentAmps = 0.0;
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
        Logger.recordOutput("Shooter/ControlPhase", phase.name());
        Logger.recordOutput("Shooter/ControlOutput", controlOutput.name());
        Logger.recordOutput("Shooter/PhaseElapsedSec", getPhaseElapsedSeconds());
        Logger.recordOutput("Shooter/LastPhaseDurationSec", lastPhaseDurationSec);
        Logger.recordOutput("Shooter/LastPhaseTransition", lastPhaseTransition);
        Logger.recordOutput("Shooter/LaunchActive", launchActive);
        Logger.recordOutput("Shooter/LaunchRisingEdge", launchRisingEdgeThisCycle);
        Logger.recordOutput("Shooter/BallEventArmed", ballEventArmed);
        Logger.recordOutput("Shooter/BallEventPending", phase == ControlPhase.BALL_PENDING);
        Logger.recordOutput("Shooter/BallEntryDetected", ballEntryDetectedThisCycle);
        Logger.recordOutput("Shooter/BallEntrySource", ballEntrySource);
        Logger.recordOutput(
                "Shooter/BallVelocityDropRPM",
                Math.min(
                        ballVelocityReferenceRPM - getAverageVelocityRPM(),
                        getVelocitySetpointRPM() - getAverageVelocityRPM()));
        Logger.recordOutput("Shooter/BallVelocityReferenceRPM", ballVelocityReferenceRPM);
        Logger.recordOutput(
                "Shooter/BallEntryDropThresholdRPM", ShooterConstants.getBallEntryDropRPM());
        Logger.recordOutput(
                "Shooter/LaunchToBallDelaySec", ShooterConstants.getLaunchToBallDelaySeconds());
        Logger.recordOutput(
                "Shooter/BallPhaseDurationSec", ShooterConstants.getBallPhaseDurationSeconds());
        Logger.recordOutput("Shooter/BallTorqueCurrentAmps", commandedTorqueCurrentAmps);
        Logger.recordOutput(
                "Shooter/StartupReadyToleranceRPM",
                ShooterConstants.getStartupReadyToleranceRPM(getVelocitySetpointRPM()));
        Logger.recordOutput(
                "Shooter/RecoveryReadyToleranceRPM",
                ShooterConstants.getRecoveryReadyToleranceRPM(getVelocitySetpointRPM()));
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
        ShooterConstants.BANG_BANG_KP.get();
    }

    public void setVelocity(double velocityRadPerSec) {
        if (manualVoltageOverride) {
            manualVoltageOverride = false;
            setPhase(ControlPhase.DISABLED, "VELOCITY_REQUESTED_AFTER_MANUAL_VOLTAGE");
        }
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

    public void setLaunchActive(boolean active) {
        launchActive = active;
        if (!active) {
            if (phase != ControlPhase.BALL && phase != ControlPhase.RECOVERY) {
                ballEventArmed = true;
            }
            previousLaunchActive = false;
        }
    }

    /** Applies open-loop voltage to all three flywheels until a velocity request or stop occurs. */
    public void runVoltage(double volts) {
        if (!allFlywheelsConnected()) {
            enabled = false;
            requestedVelocityRadPerSec = 0.0;
            manualVoltageOverride = false;
            setPhase(ControlPhase.FAULT, "MOTOR_SIGNAL_DISCONNECTED");
            stopFlywheels();
            return;
        }
        if (!Double.isFinite(volts)) {
            volts = 0.0;
        }
        double maxVolts = ShooterConstants.CHARACTERIZATION_MAX_VOLTS.getClamped(0.0, 6.0);
        double clampedVolts = Math.max(-maxVolts, Math.min(maxVolts, volts));
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = true;
        setPhase(ControlPhase.MANUAL_VOLTAGE, "CHARACTERIZATION_VOLTAGE");
        controlOutput = ControlOutput.MANUAL_VOLTAGE;
        commandedVoltage = clampedVolts;
        commandedTorqueCurrentAmps = 0.0;
        io.runVoltage(clampedVolts);
    }

    public void stop() {
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        manualVoltageOverride = false;
        launchActive = false;
        previousLaunchActive = false;
        ballEventArmed = true;
        setPhase(ControlPhase.DISABLED, "STOP_REQUESTED");
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

    private double getPhaseElapsedSeconds() {
        if (phase == ControlPhase.DISABLED || phase == ControlPhase.MANUAL_VOLTAGE) {
            return 0.0;
        }
        return Math.max(0.0, Timer.getFPGATimestamp() - phaseStartTimestampSec);
    }
}
