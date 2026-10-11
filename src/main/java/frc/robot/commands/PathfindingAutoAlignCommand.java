package frc.robot.commands;

import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Pathfinds to a Rebuilt field target, resolving alliance and target immediately before driving.
 */
public class PathfindingAutoAlignCommand extends Command {
    public static final PathConstraints pathfindConstraints =
            new PathConstraints(
                    DriveConstants.kDriveMaxSpeed * 0.8,
                            DriveConstants.kMaxAccelerationMetersPerSecondSquared * 0.8,
                    DriveConstants.kDriveMaxAngularRate * 0.8,
                            DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared * 0.8,
                    DriveConstants.kMaxXAccelerationMetersPerSecondSquared * 0.8,
                            DriveConstants.kMaxYAccelerationMetersPerSecondSquared * 0.4);
    private static final double DRIVER_OVERRIDE_DEADBAND = 0.15;
    private final Supplier<FieldLocation> locationSupplier;
    private final BooleanSupplier isRedAlliance;
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private Command innerCommand;

    public PathfindingAutoAlignCommand(
            DriveSubsystem drive,
            RobotState state,
            Supplier<FieldLocation> location,
            BooleanSupplier red) {
        this(drive, state, location, red, () -> 0.0, () -> 0.0);
    }

    public PathfindingAutoAlignCommand(
            DriveSubsystem drive,
            RobotState state,
            Supplier<FieldLocation> location,
            BooleanSupplier red,
            DoubleSupplier throttle,
            DoubleSupplier strafe) {
        locationSupplier = location;
        isRedAlliance = red;
        throttleSupplier = throttle;
        strafeSupplier = strafe;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        FieldLocation location = locationSupplier.get();
        Pose2d target = location.getPose(isRedAlliance.getAsBoolean());
        Logger.recordOutput("Auto/Pathfinding/Location", location.name());
        Logger.recordOutput("Auto/Pathfinding/TargetPose", target);
        innerCommand = AutoBuilder.pathfindToPose(target, pathfindConstraints);
        innerCommand.initialize();
    }

    @Override
    public void execute() {
        if (innerCommand != null && !innerCommand.isFinished()) innerCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return Math.hypot(throttleSupplier.getAsDouble(), strafeSupplier.getAsDouble())
                        > DRIVER_OVERRIDE_DEADBAND
                || innerCommand == null
                || innerCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (innerCommand != null) innerCommand.end(interrupted || !innerCommand.isFinished());
    }
}
