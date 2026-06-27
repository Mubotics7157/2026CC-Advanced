package frc.robot.factories;

import com.team254.lib.pathplanner.controllers.PPHolonomicDriveController;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.commands.PathfindingAutoAlignCommand;
import frc.robot.lib.commands.ChezySequenceCommandGroup;
import frc.robot.lib.reefscape.ReefBranch;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoFactory {

    public static Command getPathfindToFeederCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance) {
        return getPathfindToFeederCommand(
                driveSubsystem, robotState, letterSupplier, isRedAlliance, null);
    }

    public static Command getPathfindToFeederCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER,
                        letterSupplier,
                        () -> 0.0,
                        Commands::none,
                        () -> true,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToBargeCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.BARGE,
                        () -> false,
                        () -> 0.0,
                        Commands::none,
                        () -> true,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToFeederGroundCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER_GROUND,
                        letterSupplier,
                        () -> 0.0,
                        Commands::none,
                        () -> true,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToFeederGroundRetryCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        () -> ReefBranch.FEEDER_GROUND_RETRY,
                        letterSupplier,
                        () -> 0.0,
                        Commands::none,
                        () -> true,
                        isRedAlliance,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                inner);
    }

    public static Command getPathfindToReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            boolean shortJoinPath) {
        return getPathfindToReefCommand(
                driveSubsystem,
                robotState,
                branchSupplier,
                letterSupplier,
                heightSupplier,
                stageCommandSupplier,
                doneStagingSupplier,
                isRedAlliance,
                isAlgae,
                shortJoinPath,
                null);
    }

    public static Command getPathfindToReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            Supplier<Double> heightSupplier,
            Supplier<Command> stageCommandSupplier,
            BooleanSupplier doneStagingSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            boolean shortJoinPath,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        Supplier<Command> safeStageCommandSupplier =
                stageCommandSupplier == null ? Commands::none : stageCommandSupplier;
        BooleanSupplier safeDoneStagingSupplier =
                doneStagingSupplier == null ? () -> true : doneStagingSupplier;
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        branchSupplier,
                        letterSupplier,
                        heightSupplier,
                        safeStageCommandSupplier,
                        safeDoneStagingSupplier,
                        isRedAlliance,
                        isAlgae,
                        false,
                        shortJoinPath);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                        new InstantCommand(
                                () ->
                                        PPHolonomicDriveController.setControlPoint(
                                                DriveConstants.kDriveToCoralOffset)),
                        inner)
                .finallyDo(() -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero));
    }

    public static Command getPathfindBackoffFromReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae) {
        return getPathfindBackoffFromReefCommand(
                driveSubsystem,
                robotState,
                branchSupplier,
                letterSupplier,
                isRedAlliance,
                isAlgae,
                null);
    }

    public static Command getPathfindBackoffFromReefCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Supplier<ReefBranch> branchSupplier,
            Supplier<Boolean> letterSupplier,
            BooleanSupplier isRedAlliance,
            boolean isAlgae,
            List<PathfindingAutoAlignCommand> warmupCommands) {
        PathfindingAutoAlignCommand inner =
                new PathfindingAutoAlignCommand(
                        driveSubsystem,
                        robotState,
                        branchSupplier,
                        letterSupplier,
                        () -> 0.0,
                        Commands::none,
                        () -> true,
                        isRedAlliance,
                        isAlgae,
                        true,
                        false);
        if (warmupCommands != null) warmupCommands.add(inner);
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                new InstantCommand(() -> Pathfinding.setBackoffObstacles()),
                inner,
                new InstantCommand(() -> Pathfinding.setTeleopObstacles()));
    }
}
