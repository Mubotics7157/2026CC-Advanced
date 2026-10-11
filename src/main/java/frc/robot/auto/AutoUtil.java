package frc.robot.auto;

import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.path.PathPlannerPath;
import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Common, alliance-aware loading utilities for Choreo trajectories. */
public final class AutoUtil {
    private AutoUtil() {}

    public static List<String> getChoreoTrajectoryNames() {
        File directory = new File(Filesystem.getDeployDirectory(), "choreo");
        File[] files = directory.listFiles();
        List<String> names = new ArrayList<>();
        if (files == null) return names;
        for (File file : files) {
            if (file.isFile() && file.getName().endsWith(".traj")) {
                names.add(file.getName().substring(0, file.getName().length() - 5));
            }
        }
        names.sort(String::compareTo);
        return names;
    }

    public static boolean choreoTrajectoryExists(String name) {
        return new File(Filesystem.getDeployDirectory(), "choreo/" + name + ".traj").isFile();
    }

    /**
     * Follows a Choreo trajectory through the same configured holonomic controller used for
     * PathPlanner paths. This keeps odometry/vision corrections and telemetry consistent.
     */
    public static Command followChoreoPath(
            DriveSubsystem drive,
            RobotState robotState,
            String name,
            BooleanSupplier isRedAlliance) {
        return Commands.defer(
                        () -> {
                            try {
                                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
                                Logger.recordOutput("Auto/Choreo/Path", name);
                                return AutoBuilder.followPath(path);
                            } catch (Exception exception) {
                                DriverStation.reportError(
                                        "Unable to load Choreo path " + name,
                                        exception.getStackTrace());
                                Logger.recordOutput("Auto/Choreo/LoadError", exception.toString());
                                return Commands.none();
                            }
                        },
                        java.util.Set.of(drive))
                .withName("Choreo: " + name);
    }

    public static Optional<Pose2d> getChoreoStartingPose(String name, boolean isRedAlliance) {
        try {
            Optional<Pose2d> pose =
                    PathPlannerPath.fromChoreoTrajectory(name).getStartingHolonomicPose();
            return pose.map(value -> isRedAlliance ? FlippingUtil.flipFieldPose(value) : value);
        } catch (Exception exception) {
            Logger.recordOutput("Auto/Choreo/StartingPoseError", exception.toString());
            return Optional.empty();
        }
    }
}
