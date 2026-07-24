package frc.robot.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

/**
 * Factory for creating shooter-related commands following Team 254's pattern. Static factory
 * methods for coordinating shooting operations between superstructure and shooter subsystem.
 */
public final class ShooterFactory {

    private ShooterFactory() {}

    /**
     * Command to set the superstructure to SHOOTING state. Runs continuously - use with whileTrue()
     * bindings. Returns to IDLE when interrupted.
     *
     * @param superstructure The superstructure subsystem
     * @return Command that maintains SHOOTING state while running
     */
    public static Command setShootingCommand(Superstructure superstructure) {
        return superstructure
                .startEnd(
                        () -> superstructure.setGoal(Goal.SHOOTING),
                        () -> superstructure.setGoal(Goal.DEPLOYED_IDLE))
                .withName("setShootingCommand");
    }

    // TODO: IMPLEMENT FERRY SHOT WITH CC STUFF
    // /**
    //  * Command to ferry shoot toward the alliance corner. Switches robot target to the alliance
    //  * corner, enters SHOOTING state, and restores normal movement when interrupted.
    //  *
    //  * @param superstructure The superstructure subsystem
    //  * @return Command that ferry shoots while tracking the alliance corner
    //  */
    // public static Command ferryShotCommand(
    //         Superstructure superstructure ) {
    //     return Commands.sequence(
    //                     Commands.runOnce(turretManager::trackAllianceCorner),
    //                     superstructure.run(() -> superstructure.setGoal(Goal.SHOOTING)))
    //             .finallyDo(
    //                     () -> {
    //                         superstructure.setGoal(Goal.DEPLOYED_IDLE);
    //                         turretManager.trackHub();
    //                     })
    //             .withName("ferryShotCommand");
    // }
}
