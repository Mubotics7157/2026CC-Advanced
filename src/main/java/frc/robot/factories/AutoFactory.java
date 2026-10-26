package frc.robot.factories;

import com.team254.lib.pathplanner.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.commands.PathfindingAutoAlignCommand;
import frc.robot.lib.commands.ChezySequenceCommandGroup;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Rebuilt-specific factory for all alliance-aware pathfinding commands. */
public final class AutoFactory {
    private AutoFactory() {}

    public static Command pathfindTo(
            DriveSubsystem drive,
            RobotState state,
            Supplier<FieldLocation> target,
            BooleanSupplier red) {
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                PathfindingAutoAlignCommand.create(drive, target, red));
    }

    public static Command pathfindTo(
            DriveSubsystem drive,
            RobotState state,
            Supplier<FieldLocation> target,
            BooleanSupplier red,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        return new ChezySequenceCommandGroup(
                new InstantCommand(
                        () -> PPHolonomicDriveController.setControlPoint(Transform2d.kZero)),
                PathfindingAutoAlignCommand.create(drive, target, red, throttle, strafe));
    }

    public static Command getPathfindToDepotCommand(
            DriveSubsystem drive, RobotState state, BooleanSupplier red) {
        return pathfindTo(drive, state, () -> FieldLocation.DEPOT, red);
    }

    public static Command getPathfindToLeftCommand(
            DriveSubsystem drive, RobotState state, BooleanSupplier red) {
        return pathfindTo(drive, state, () -> FieldLocation.LEFT, red);
    }

    public static Command getPathfindToRightCommand(
            DriveSubsystem drive, RobotState state, BooleanSupplier red) {
        return pathfindTo(drive, state, () -> FieldLocation.RIGHT, red);
    }

    public static Command getPathfindToHubCommand(
            DriveSubsystem drive,
            RobotState state,
            BooleanSupplier red,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        return pathfindTo(drive, state, () -> FieldLocation.HUB_NEAR, red, throttle, strafe);
    }
}
