package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
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
    private final Feeder feeder;
    private final Shooter shooter;

    private Goal currentGoal = null;
    private Goal desiredGoal = null;
    private boolean feedingLatched = false;
    private double shooterReadyTimestamp = 0.0;

    public Superstructure(Intake intake, Indexer indexer, Feeder feeder, Shooter shooter) {
        this.intake = intake;
        this.indexer = indexer;
        this.feeder = feeder;
        this.shooter = shooter;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Superstructure/CurrentGoal", currentGoal == null ? "IDLE" : currentGoal.name());
        Logger.recordOutput(
                "Superstructure/DesiredGoal", desiredGoal == null ? "IDLE" : desiredGoal.name());
        Logger.recordOutput("Superstructure/AtGoal", atGoal());
        Logger.recordOutput("Superstructure/GoalTransitioning", currentGoal != desiredGoal);

        Logger.recordOutput(
                "Superstructure/IntakeExtensionPositionMeters",
                intake.getExtensionPositionMeters());
        Logger.recordOutput(
                "Superstructure/IntakeRollerVelocityRadPerSec",
                intake.getRollerVelocityRadPerSec());
        Logger.recordOutput(
                "Superstructure/IndexerConveyorVelocityRadPerSec",
                indexer.getConveyorVelocityRadPerSec());
        Logger.recordOutput("Superstructure/IndexerRunning", indexer.isRunning());
        Logger.recordOutput(
                "Superstructure/FeederVelocityRadPerSec", feeder.getFeederVelocityRadPerSec());
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
                intake.extend();
                intake.setRollerVelocity(
                        SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                indexer.setConveyorVelocity(
                        SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                feeder.stop();
                shooter.stop();
                break;
            case OUTTAKING:
                intake.extend();
                intake.setRollerVelocity(
                        -SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                indexer.setConveyorVelocity(
                        -SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                feeder.setFeederVelocity(-SuperstructureConstants.FEEDER_VELOCITY_RAD_PER_SEC);
                shooter.stop();
                break;
            case SHOOTING:
                intake.retract();
                intake.setRollerVelocity(0.0);
                shooter.setVelocity(SuperstructureConstants.SHOOTER_VELOCITY_RAD_PER_SEC);
                if (shooter.atSetpoint()) {
                    if (shooterReadyTimestamp == 0.0) {
                        shooterReadyTimestamp = Timer.getFPGATimestamp();
                    }
                    feedingLatched =
                            Timer.getFPGATimestamp() - shooterReadyTimestamp
                                    >= SuperstructureConstants.SHOOTER_FEED_DELAY_SECONDS;
                } else {
                    shooterReadyTimestamp = 0.0;
                }
                if (feedingLatched) {
                    indexer.setConveyorVelocity(
                            SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                    feeder.setFeederVelocity(SuperstructureConstants.FEEDER_VELOCITY_RAD_PER_SEC);
                } else {
                    indexer.stop();
                    feeder.stop();
                }
                break;
            case IDLE:
                intake.stopRoller();
                intake.retract();
                shooter.stop();
                feeder.stop();
                indexer.stop();
            break;
            case DEPLOYED_IDLE:
                intake.stopRoller();
                intake.extend();
                shooter.stop();
                feeder.stop();
                indexer.stop();
        }
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

    public boolean atGoal() {
        if (currentGoal != desiredGoal) {
            return false;
        }
        if (currentGoal == null) {
            return true;
        }

        return switch (currentGoal) {
            case INTAKING -> intake.isExtended() && indexer.isRunning();
            case OUTTAKING -> intake.isExtended() && indexer.isRunning() && feeder.isRunning();
            case SHOOTING -> intake.isRetracted() && shooter.atSetpoint() && feedingLatched;
        };
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal).withName("Superstructure Wait At Goal");
    }

    private void commandIdle() {
        intake.retract();
        intake.setRollerVelocity(0.0);
        indexer.stop();
        feeder.stop();
        shooter.stop();
    }
}
