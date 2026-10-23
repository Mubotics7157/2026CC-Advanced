package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.factories.AutoFactory;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Composes a game-aware autonomous route from adaptive legs. Each pathfinding leg resolves its
 * target at initialize time, so it uses the latest fused odometry and any accepted vision updates.
 */
public final class AdaptiveAutoBuilder {
    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final BooleanSupplier isRedAlliance;
    private Supplier<FieldLocation> firstTarget = () -> FieldLocation.DEPOT;
    private Supplier<FieldLocation> secondTarget = () -> FieldLocation.HUB_NEAR;
    private Supplier<FieldLocation> thirdTarget = () -> FieldLocation.DEPOT;
    private boolean visitFirstTarget;
    private boolean visitSecondTarget;
    private boolean visitThirdTarget;
    private Command firstTargetAction = Commands.none();
    private Command secondTargetAction = Commands.none();
    private Command thirdTargetAction = Commands.none();

    public AdaptiveAutoBuilder(
            DriveSubsystem drive, RobotState robotState, BooleanSupplier isRedAlliance) {
        this.drive = drive;
        this.robotState = robotState;
        this.isRedAlliance = isRedAlliance;
    }

    public AdaptiveAutoBuilder targets(
            Supplier<FieldLocation> first,
            Supplier<FieldLocation> second,
            Supplier<FieldLocation> third) {
        firstTarget = first;
        secondTarget = second;
        thirdTarget = third;
        return this;
    }

    public AdaptiveAutoBuilder visitFirstTarget(boolean value) {
        visitFirstTarget = value;
        return this;
    }

    public AdaptiveAutoBuilder visitSecondTarget(boolean value) {
        visitSecondTarget = value;
        return this;
    }

    public AdaptiveAutoBuilder visitThirdTarget(boolean value) {
        visitThirdTarget = value;
        return this;
    }

    public AdaptiveAutoBuilder afterFirstTarget(Command action) {
        firstTargetAction = action;
        return this;
    }

    public AdaptiveAutoBuilder afterSecondTarget(Command action) {
        secondTargetAction = action;
        return this;
    }

    public AdaptiveAutoBuilder afterThirdTarget(Command action) {
        thirdTargetAction = action;
        return this;
    }

    public Command build() {
        Command route =
                Commands.runOnce(
                        () ->
                                Logger.recordOutput(
                                        "Auto/Adaptive/InitialPose",
                                        robotState.getLatestFieldToRobot().getValue()));
        if (visitFirstTarget)
            route =
                    route.andThen(
                            AutoFactory.pathfindTo(drive, robotState, firstTarget, isRedAlliance),
                            firstTargetAction);
        if (visitSecondTarget)
            route =
                    route.andThen(
                            AutoFactory.pathfindTo(drive, robotState, secondTarget, isRedAlliance),
                            secondTargetAction);
        if (visitThirdTarget)
            route =
                    route.andThen(
                            AutoFactory.pathfindTo(drive, robotState, thirdTarget, isRedAlliance),
                            thirdTargetAction);
        return route.withName("Adaptive Autonomous Route");
    }
}
