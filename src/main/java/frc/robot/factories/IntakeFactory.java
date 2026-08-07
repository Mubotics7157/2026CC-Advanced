package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

/**
 * Factory for creating intake-related commands following Team 254's pattern. Static factory methods
 * for goal-based state machine commands.
 */
public final class IntakeFactory {

    private IntakeFactory() {
        // Utility class - no instantiation
    }

    /**
     * Command to set the superstructure to INTAKING state. Runs continuously - use with whileTrue()
     * bindings. Returns to DEPLOYED_IDLE when interrupted (arm stays out, rollers stop).
     *
     * @param superstructure The superstructure subsystem
     * @return Command that maintains INTAKING state while running
     */
    public static Command setIntakingCommand(Superstructure superstructure) {
        return superstructure
                .startEnd(
                        () -> superstructure.setGoal(Goal.INTAKING),
                        () -> superstructure.setGoal(Goal.DEPLOYED_IDLE))
                .withName("setIntakingCommand");
    }

    /**
     * Command to set the superstructure to OUTTAKING state. Runs continuously - use with
     * whileTrue() bindings. Returns to IDLE when interrupted.
     *
     * @param superstructure The superstructure subsystem
     * @return Command that maintains OUTTAKING state while running
     */
    public static Command setOuttakingCommand(Superstructure superstructure) {
        return superstructure
                .startEnd(
                        () -> superstructure.setGoal(Goal.OUTTAKING),
                        () -> superstructure.setGoal(Goal.DEPLOYED_IDLE))
                .withName("setOuttakingCommand");
    }

    /**
     * Runs the complete shoot sequence while held. Releasing it returns the forebar to its safe
     * stowed idle state.
     */
    public static Command setShootingCommand(Superstructure superstructure) {
        return superstructure
                .startEnd(
                        () -> superstructure.setGoal(Goal.SHOOTING),
                        () -> superstructure.setGoal(Goal.IDLE))
                .withName("setShootingCommand");
    }

    /**
     * Driver manual-stow command: while held, forces the superstructure to IDLE (forebar stowed,
     * rollers off). The intake subsystem requirement allows another intake action to interrupt it.
     *
     * @param superstructure The superstructure subsystem
     * @return Command that maintains IDLE while running
     */
    public static Command manualStowCommand(Superstructure superstructure) {
        return superstructure
                .startEnd(
                        () -> superstructure.setGoal(Goal.IDLE),
                        () -> superstructure.setGoal(Goal.IDLE))
                .withName("manualStowCommand");
    }

    /**
     * Command to set the superstructure to IDLE state immediately.
     *
     * @param superstructure The superstructure subsystem
     * @return Instant command that sets IDLE goal
     */
    public static Command setIdleCommand(Superstructure superstructure) {
        return superstructure
                .runOnce(() -> superstructure.setGoal(Goal.IDLE))
                .withName("setIdleCommand");
    }
}
