package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
     * Driver manual-stow command: while held, forces the superstructure to IDLE (arm stowed,
     * rollers off) and drives both turrets to their side-specific aim zero offset positions. When
     * released, the superstructure stays IDLE and the turrets fall back to the default hub
     * tracking. If another input that requires the superstructure (e.g., intake) is pressed while
     * this command is running, the subsystem requirement causes this command to be interrupted and
     * the other input takes precedence.
     *
     * @param superstructure The superstructure subsystem
     * @param turretManager The turret manager subsystem
     * @return Command that forces IDLE + zero-offset turrets while running
     */
    public static Command manualStowCommand(
            Superstructure superstructure) {
        return Commands.parallel(
                        superstructure.startEnd(
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
