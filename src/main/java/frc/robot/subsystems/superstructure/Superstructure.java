package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.rebuilt.RebuiltHubStatus;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    public enum Goal {
        INTAKING,
        OUTTAKING,
        SHOOTING,
        IDLE,
        DEPLOYED_IDLE
    }

    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;

    private Goal currentGoal = null;
    private Goal desiredGoal = null;
    private boolean feedingLatched = false;
    private double shooterReadyTimestamp = 0.0;
    private double shootingVelocityRadPerSec = SuperstructureConstants.SHOOTER_VELOCITY_RAD_PER_SEC;
    private double shootingHoodAngleRad = SuperstructureConstants.SHOOTER_HOOD_ANGLE_RAD;
    private boolean shotReadyToFeed = true;

    // "Dumb mode": when active and SHOOTING, bypass tolerance/setpoint gating and feed
    // immediately regardless of whether the hood, flywheels, and targeting solution have
    // reached their setpoints.
    // Toggled by the driver via the control board; persists across SHOOTING entries until
    // toggled off.
    private boolean dumbModeEnabled = false;
    private boolean shooterTuningOverride = false;
    private boolean ownHubActive;

    public Superstructure(Intake intake, Indexer indexer, Shooter shooter) {
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
    }

    @Override
    public void periodic() {
        ownHubActive = RebuiltHubStatus.isOwnHubActive();
        Logger.recordOutput(
                "Superstructure/CurrentGoal", currentGoal == null ? "IDLE" : currentGoal.name());
        Logger.recordOutput(
                "Superstructure/DesiredGoal", desiredGoal == null ? "IDLE" : desiredGoal.name());
        Logger.recordOutput("Superstructure/AtGoal", atGoal());
        Logger.recordOutput("Superstructure/GoalTransitioning", currentGoal != desiredGoal);

        Logger.recordOutput("Superstructure/IntakeArmPositionRad", intake.getArmPositionRad());
        Logger.recordOutput(
                "Superstructure/IntakeRollerVelocityRadPerSec",
                intake.getRollerVelocityRadPerSec());
        Logger.recordOutput(
                "Superstructure/IndexerConveyorVelocityRadPerSec",
                indexer.getConveyorVelocityRadPerSec());
        Logger.recordOutput("Superstructure/IndexerRunning", indexer.isRunning());
        Logger.recordOutput(
                "Superstructure/FeederVelocityRadPerSec", indexer.getFeederVelocityRadPerSec());
        Logger.recordOutput(
                "Superstructure/ShooterLeftVelocityRadPerSec", shooter.getLeftVelocityRadPerSec());
        Logger.recordOutput(
                "Superstructure/ShooterMiddleVelocityRadPerSec",
                shooter.getMiddleVelocityRadPerSec());
        Logger.recordOutput(
                "Superstructure/ShooterRightVelocityRadPerSec",
                shooter.getRightVelocityRadPerSec());
        Logger.recordOutput("Superstructure/ShooterAtSetpoint", shooter.atSetpoint());
        Logger.recordOutput("Superstructure/FeedingLatched", feedingLatched);
        Logger.recordOutput("Superstructure/OwnHubActive", ownHubActive);
        Logger.recordOutput("Superstructure/DumbModeEnabled", dumbModeEnabled);
        Logger.recordOutput("Superstructure/ShootingVelocityRadPerSec", shootingVelocityRadPerSec);
        Logger.recordOutput("Superstructure/ShootingHoodAngleRad", shootingHoodAngleRad);
        Logger.recordOutput(
                "Superstructure/ShootingHoodAngleDeg", Math.toDegrees(shootingHoodAngleRad));
        Logger.recordOutput("Superstructure/ShotReadyToFeed", shotReadyToFeed);
        Logger.recordOutput("Superstructure/ShooterTuningOverride", shooterTuningOverride);

        Logger.recordOutput("Superstructure/TuningMode", Constants.kTuningMode);
        Logger.recordOutput(
                "Tuning/Intake/RollerSetpointRadPerSec",
                TuningSetpoints.INTAKE_ROLLER_RAD_PER_SEC.get());
        Logger.recordOutput("Tuning/Intake/ArmSetpointRad", TuningSetpoints.INTAKE_ARM_RAD.get());
        Logger.recordOutput(
                "Tuning/Indexer/ConveyorSetpointRadPerSec",
                TuningSetpoints.INDEXER_CONVEYOR_RAD_PER_SEC.get());
        Logger.recordOutput(
                "Tuning/Indexer/FeederSetpointRadPerSec",
                TuningSetpoints.INDEXER_FEEDER_RAD_PER_SEC.get());
        Logger.recordOutput(
                "Tuning/Shooter/FlywheelSetpointRadPerSec",
                TuningSetpoints.shooterVelocityRadPerSec());
        Logger.recordOutput("Tuning/Shooter/HoodSetpointRad", TuningSetpoints.hoodAngleRad());

        if (Constants.kTuningMode) {
            applyTuningSetpoints();
            return;
        }

        if (shooterTuningOverride) {
            intake.stopRoller();
            indexer.stop();
            return;
        }

        if (currentGoal != desiredGoal
                && (currentGoal == Goal.SHOOTING || desiredGoal == Goal.SHOOTING)) {
            feedingLatched = false;
            shooterReadyTimestamp = 0.0;
        }

        currentGoal = desiredGoal;

        if (currentGoal == null) {
            commandIdle();
            return;
        }

        switch (currentGoal) {
            case INTAKING:
                intake.deployArm();
                intake.setRollerVelocity(
                        SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                indexer.setConveyorVelocity(
                        SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                indexer.stopFeeder();
                shooter.stop();
                break;
            case OUTTAKING:
                intake.deployArm();
                intake.setRollerVelocity(
                        -SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                indexer.setConveyorVelocity(
                        -SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                indexer.setFeederVelocity(-SuperstructureConstants.FEEDER_VELOCITY_RAD_PER_SEC);
                shooter.stop();
                break;
            case SHOOTING:
                shooter.setVelocity(shootingVelocityRadPerSec);
                shooter.setHoodAngle(shootingHoodAngleRad);

                // Normal gating updates feedingLatched in the background so behavior reverts
                // cleanly
                // if dumb mode is toggled off mid-shot.
                if (!feedingLatched) {
                    if (shooter.atSetpoint() && shotReadyToFeed) {
                        if (shooterReadyTimestamp == 0.0) {
                            shooterReadyTimestamp = Timer.getFPGATimestamp();
                        }
                        if (Timer.getFPGATimestamp() - shooterReadyTimestamp
                                >= SuperstructureConstants.SHOOTER_FEED_DELAY_SECONDS) {
                            feedingLatched = true;
                        }
                    } else {
                        shooterReadyTimestamp = 0.0;
                    }
                }

                // Dumb mode short-circuits the gate entirely: feeders run immediately on
                // entry to SHOOTING regardless of tolerances or setpoints. The latch keeps
                // updating in the background so behavior reverts cleanly if dumb mode is
                // toggled off mid-shot.
                boolean shouldFeed = ownHubActive && (feedingLatched || dumbModeEnabled);
                shooter.setLaunchActive(shouldFeed);
                if (shouldFeed) {
                    // Only use the intake to agitate balls once feeding has actually started.
                    // In normal mode this follows the shooter/aim readiness gate; dumb mode
                    // reaches this branch as soon as its feed is enabled.
                    commandShootingAgitation();
                    intake.setRollerVelocity(
                            SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                    indexer.setConveyorVelocity(
                            SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                    indexer.setFeederVelocity(SuperstructureConstants.FEEDER_VELOCITY_RAD_PER_SEC);
                } else {
                    intake.deployArm();
                    intake.stopRoller();
                    indexer.stop();
                }
                break;
            case IDLE:
                intake.stopRoller();
                intake.stowArm();
                commandShooterStowed();
                indexer.stop();
                break;
            case DEPLOYED_IDLE:
                intake.stopRoller();
                intake.deployArm();
                commandShooterStowed();
                indexer.stop();
                break;
        }
    }

    private void applyTuningSetpoints() {
        intake.setArmPosition(TuningSetpoints.INTAKE_ARM_RAD.get());
        intake.setRollerVelocity(TuningSetpoints.INTAKE_ROLLER_RAD_PER_SEC.get());
        indexer.setConveyorVelocity(TuningSetpoints.INDEXER_CONVEYOR_RAD_PER_SEC.get());
        indexer.setFeederVelocity(TuningSetpoints.INDEXER_FEEDER_RAD_PER_SEC.get());
        shooter.setLaunchActive(false);
        shooter.setVelocity(TuningSetpoints.shooterVelocityRadPerSec());
        shooter.setHoodAngle(TuningSetpoints.hoodAngleRad());
    }

    public Goal getCurrentGoal() {
        return currentGoal;
    }

    public Goal getDesiredGoal() {
        return desiredGoal;
    }

    public void setGoal(Goal goal) {
        desiredGoal = goal;
    }

    public void stop() {
        desiredGoal = null;
    }

    public void setShootingRequest(
            double velocityRadPerSec, double hoodAngleRad, boolean readyToFeed) {
        shootingVelocityRadPerSec = velocityRadPerSec;
        shootingHoodAngleRad = hoodAngleRad;
        shotReadyToFeed = readyToFeed;
    }

    public void clearShootingRequest() {
        shootingVelocityRadPerSec = SuperstructureConstants.SHOOTER_VELOCITY_RAD_PER_SEC;
        shootingHoodAngleRad = SuperstructureConstants.SHOOTER_HOOD_ANGLE_RAD;
        shotReadyToFeed = true;
    }

    /**
     * Temporarily gives a tuning command exclusive control of the shooter; feeding stays stopped.
     */
    public void setShooterTuningOverride(boolean enabled) {
        boolean wasEnabled = shooterTuningOverride;
        shooterTuningOverride = enabled;
        if (enabled) {
            feedingLatched = false;
            shooterReadyTimestamp = 0.0;
            shooter.setLaunchActive(false);
            indexer.stop();
            if (!wasEnabled) {
                shooter.stop();
            }
        }
    }

    /**
     * Driver "dumb mode" — when enabled, the SHOOTING state bypasses tolerance/setpoint gating and
     * feeds balls immediately on entry, regardless of whether the hood, flywheels, and solution
     * have reached their setpoints. Toggled by the driver via the control board.
     *
     * @return True if dumb mode is currently active.
     */
    public boolean isDumbModeEnabled() {
        return dumbModeEnabled;
    }

    /** Set dumb mode explicitly. See {@link #isDumbModeEnabled()}. */
    public void setDumbModeEnabled(boolean enabled) {
        dumbModeEnabled = enabled;
    }

    /** Toggle dumb mode on/off. Intended for an .onTrue() driver button binding. */
    public void toggleDumbMode() {
        dumbModeEnabled = !dumbModeEnabled;
    }

    public boolean atGoal() {
        if (currentGoal != desiredGoal) {
            return false;
        }
        if (currentGoal == null) {
            return true;
        }

        return switch (currentGoal) {
            case INTAKING -> intake.isDeployed() && indexer.isRunning();
            case OUTTAKING -> intake.isDeployed()
                    && indexer.isConveyorRunning()
                    && indexer.isFeederRunning();
            case SHOOTING -> ownHubActive && (feedingLatched || dumbModeEnabled);
            case IDLE -> intake.isStowed();
            case DEPLOYED_IDLE -> intake.isDeployed();
        };
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal).withName("Superstructure Wait At Goal");
    }

    private void commandIdle() {
        intake.stowArm();
        intake.setRollerVelocity(0.0);
        indexer.stop();
        commandShooterStowed();
    }

    private void commandShootingAgitation() {
        double phase =
                (Timer.getFPGATimestamp()
                                % SuperstructureConstants.SHOOTING_AGITATION_PERIOD_SECONDS)
                        / SuperstructureConstants.SHOOTING_AGITATION_PERIOD_SECONDS;
        if (phase < 0.5) {
            intake.setArmPosition(IntakeConstants.ARM_AGITATE_POSITION_RAD);
        } else {
            intake.setArmPosition(IntakeConstants.ARM_DEPLOYED_POSITION_RAD);
        }
    }

    /** Stops the flywheels and positively parks the hood at its safe, lowest angle. */
    private void commandShooterStowed() {
        shooter.stop();
        shooter.setHoodAngle(ShooterConstants.HOOD_MIN_ANGLE_RAD);
    }
}
