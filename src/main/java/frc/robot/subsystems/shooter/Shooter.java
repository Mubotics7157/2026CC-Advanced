package frc.robot.subsystems.shooter;

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
        RECOVERY
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double requestedVelocityRadPerSec = 0.0;
    private boolean enabled = false;
    private boolean launchActive = false;
    private boolean previousLaunchActive = false;
    private double previousVelocityRadPerSec = 0.0;
    private double phaseStartTimestampSec = 0.0;
    private ControlPhase phase = ControlPhase.DISABLED;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (ShooterConstants.hasAnyShooterPIDChanged()) {
            io.updateShooterConfig();
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
        previousVelocityRadPerSec = averageFlywheelVelocity();
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
                                && previousVelocityRadPerSec - averageFlywheelVelocity()
                                        > rpmToRadPerSec(
                                                ShooterConstants.BALL_ENTRY_DROP_RPM.get());
                if (launchRisingEdge || ballEntryByVelocityDrop) {
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
        requestedVelocityRadPerSec =
                Math.max(
                        0.0,
                        Math.min(
                                velocityRadPerSec,
                                ShooterConstants.SHOOTER_MAX_VELOCITY_RAD_PER_SEC));
        enabled = requestedVelocityRadPerSec > 1e-3;
        if (!enabled) {
            stop();
        }
    }

    public void setHoodAngle(double angleRad) {
        io.setHoodServoPosition(ShooterConstants.hoodAngleToServoPosition(angleRad));
    }

    public void setLaunchActive(boolean active) {
        launchActive = active;
    }

    public void stop() {
        enabled = false;
        requestedVelocityRadPerSec = 0.0;
        launchActive = false;
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

    public double getVelocitySetpointRadPerSec() {
        return requestedVelocityRadPerSec;
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
        if (requestedVelocityRadPerSec <= ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC) {
            return false;
        }
        double tolerance = ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC;
        return Math.abs(requestedVelocityRadPerSec - inputs.leftVelocity) < tolerance
                && Math.abs(requestedVelocityRadPerSec - inputs.middleVelocity) < tolerance
                && Math.abs(requestedVelocityRadPerSec - inputs.rightVelocity) < tolerance;
    }

    public boolean hoodAtSetpoint() {
        return inputs.hoodAtSetpoint;
    }

    public boolean isRunning() {
        double tolerance = ShooterConstants.VELOCITY_TOLERANCE_RAD_PER_SEC;
        return Math.abs(inputs.leftVelocity) > tolerance
                || Math.abs(inputs.middleVelocity) > tolerance
                || Math.abs(inputs.rightVelocity) > tolerance;
    }
}
