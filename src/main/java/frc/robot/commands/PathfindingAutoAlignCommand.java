package frc.robot.commands;

import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Pathfinds to a Rebuilt field target, resolving alliance and target immediately before driving.
 */
public final class PathfindingAutoAlignCommand {
    public static final PathConstraints pathfindConstraints =
            new PathConstraints(
                    DriveConstants.kDriveMaxSpeed * 0.8,
                            DriveConstants.kMaxAccelerationMetersPerSecondSquared * 0.8,
                    DriveConstants.kDriveMaxAngularRate * 0.8,
                            DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared * 0.8,
                    DriveConstants.kMaxXAccelerationMetersPerSecondSquared * 0.8,
                            DriveConstants.kMaxYAccelerationMetersPerSecondSquared * 0.4);
    private static final double DRIVER_OVERRIDE_DEADBAND = 0.15;

    private PathfindingAutoAlignCommand() {}

    public static Command create(
            DriveSubsystem drive, Supplier<FieldLocation> location, BooleanSupplier red) {
        return create(drive, location, red, () -> 0.0, () -> 0.0);
    }

    public static Command create(
            DriveSubsystem drive,
            Supplier<FieldLocation> location,
            BooleanSupplier red,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        return Commands.defer(
                        () -> {
                            FieldLocation selectedLocation = location.get();
                            Pose2d target = selectedLocation.getPose(red.getAsBoolean());
                            Logger.recordOutput(
                                    "Auto/Pathfinding/Location", selectedLocation.name());
                            Logger.recordOutput("Auto/Pathfinding/TargetPose", target);
                            return AutoBuilder.pathfindToPose(target, pathfindConstraints)
                                    .until(
                                            () ->
                                                    Math.hypot(
                                                                    throttle.getAsDouble(),
                                                                    strafe.getAsDouble())
                                                            > DRIVER_OVERRIDE_DEADBAND);
                        },
                        Set.of(drive))
                .withName("Pathfind To Location");
    }
}
