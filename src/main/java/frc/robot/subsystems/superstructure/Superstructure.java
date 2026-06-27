package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    public enum Goal {
        INTAKING,
        OUTTAKING,
        SHOOTING
    }

    private final Intake intake;
    private final Indexer indexer;

    private Goal currentGoal = null;
    private Goal desiredGoal = null;

    public Superstructure(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
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
                break;
            case OUTTAKING:
                intake.extend();
                intake.setRollerVelocity(
                        -SuperstructureConstants.INTAKE_ROLLER_VELOCITY_RAD_PER_SEC);
                indexer.setConveyorVelocity(
                        -SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                break;
            case SHOOTING:
                intake.retract();
                intake.setRollerVelocity(0.0);
                indexer.setConveyorVelocity(
                        SuperstructureConstants.INDEXER_CONVEYOR_VELOCITY_RAD_PER_SEC);
                break;
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
            case INTAKING, OUTTAKING -> intake.isExtended() && indexer.isRunning();
            case SHOOTING -> intake.isRetracted() && indexer.isRunning();
        };
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal).withName("Superstructure Wait At Goal");
    }

    private void commandIdle() {
        intake.retract();
        intake.setRollerVelocity(0.0);
        indexer.stop();
    }
}
