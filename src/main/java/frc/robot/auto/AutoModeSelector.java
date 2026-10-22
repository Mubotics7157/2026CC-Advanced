package frc.robot.auto;

import com.team254.lib.pathplanner.commands.PathPlannerAuto;
import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.factories.AutoFactory;
import frc.robot.lib.rebuilt.FieldLocation;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Dashboard configuration for all Rebuilt file, Choreo, and adaptive autonomous routines. */
public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        SYSID_QUASISTATIC_FORWARD,
        SYSID_QUASISTATIC_REVERSE,
        SYSID_DYNAMIC_FORWARD,
        SYSID_DYNAMIC_REVERSE,
        PATHPLANNER_AUTO,
        CHOREO_PATH,
        ADAPTIVE_ROUTE,
        PATHFIND_TO_LOCATION
    }

    private final RobotContainer container;
    private final LoggedDashboardChooser<DesiredMode> modeChooser =
            new LoggedDashboardChooser<>("Auto/Mode");
    private final LoggedDashboardChooser<String> pathPlannerChooser =
            new LoggedDashboardChooser<>("Auto/PathPlannerFile");
    private final LoggedDashboardChooser<String> choreoChooser =
            new LoggedDashboardChooser<>("Auto/ChoreoTrajectory");
    private final LoggedDashboardChooser<FieldLocation> targetChooser =
            new LoggedDashboardChooser<>("Auto/PathfindTarget");
    private final LoggedDashboardChooser<FieldLocation> adaptiveFirstChooser =
            new LoggedDashboardChooser<>("Auto/Adaptive/FirstTarget");
    private final LoggedDashboardChooser<FieldLocation> adaptiveSecondChooser =
            new LoggedDashboardChooser<>("Auto/Adaptive/SecondTarget");
    private final LoggedDashboardChooser<FieldLocation> adaptiveThirdChooser =
            new LoggedDashboardChooser<>("Auto/Adaptive/ThirdTarget");
    private DesiredMode lastMode;
    private String lastPath;
    private String lastChoreo;
    private FieldLocation lastTarget;
    private FieldLocation lastAdaptiveFirst;
    private FieldLocation lastAdaptiveSecond;
    private FieldLocation lastAdaptiveThird;
    private Optional<DriverStation.Alliance> lastAlliance = Optional.empty();

    public AutoModeSelector(RobotContainer container) {
        this.container = container;
        modeChooser.addDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser.addOption(
                "Drive SysId (Quasistatic Forward)", DesiredMode.SYSID_QUASISTATIC_FORWARD);
        modeChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", DesiredMode.SYSID_QUASISTATIC_REVERSE);
        modeChooser.addOption("Drive SysId (Dynamic Forward)", DesiredMode.SYSID_DYNAMIC_FORWARD);
        modeChooser.addOption("Drive SysId (Dynamic Reverse)", DesiredMode.SYSID_DYNAMIC_REVERSE);
        modeChooser.addOption("PathPlanner File Auto", DesiredMode.PATHPLANNER_AUTO);
        modeChooser.addOption("Choreo Trajectory", DesiredMode.CHOREO_PATH);
        modeChooser.addOption("Adaptive Route", DesiredMode.ADAPTIVE_ROUTE);
        modeChooser.addOption("Pathfind to Rebuilt Location", DesiredMode.PATHFIND_TO_LOCATION);
        pathPlannerChooser.addDefaultOption("None", "");
        for (String name : com.team254.lib.pathplanner.auto.AutoBuilder.getAllAutoNames())
            pathPlannerChooser.addOption(name, name);
        choreoChooser.addDefaultOption("None", "");
        for (String name : AutoUtil.getChoreoTrajectoryNames()) choreoChooser.addOption(name, name);
        configureLocationChooser(targetChooser, FieldLocation.HUB_NEAR);
        configureLocationChooser(adaptiveFirstChooser, FieldLocation.DEPOT);
        configureLocationChooser(adaptiveSecondChooser, FieldLocation.HUB_NEAR);
        configureLocationChooser(adaptiveThirdChooser, FieldLocation.DEPOT);
    }

    private static void configureLocationChooser(
            LoggedDashboardChooser<FieldLocation> chooser, FieldLocation defaultValue) {
        chooser.addDefaultOption(displayName(defaultValue), defaultValue);
        for (FieldLocation location : FieldLocation.values())
            if (location != defaultValue) chooser.addOption(displayName(location), location);
    }

    private static String displayName(FieldLocation location) {
        return location.name().replace('_', ' ');
    }

    public Command getAutonomousCommand() {
        DesiredMode mode = Optional.ofNullable(modeChooser.get()).orElse(DesiredMode.DO_NOTHING);
        BooleanSupplier red = container.getRobotState()::isRedAlliance;
        var drive = container.getDriveSubsystem();
        var state = container.getRobotState();
        Logger.recordOutput("Auto/SelectedMode", mode.name());
        return switch (mode) {
            case DO_NOTHING -> Commands.none();
            case SYSID_QUASISTATIC_FORWARD -> drive.sysIdQuasistatic(
                            SysIdRoutine.Direction.kForward)
                    .withName("Drive SysId Quasistatic Forward");
            case SYSID_QUASISTATIC_REVERSE -> drive.sysIdQuasistatic(
                            SysIdRoutine.Direction.kReverse)
                    .withName("Drive SysId Quasistatic Reverse");
            case SYSID_DYNAMIC_FORWARD -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
                    .withName("Drive SysId Dynamic Forward");
            case SYSID_DYNAMIC_REVERSE -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                    .withName("Drive SysId Dynamic Reverse");
            case PATHPLANNER_AUTO -> loadPathPlannerAuto(pathPlannerChooser.get());
            case CHOREO_PATH -> AutoUtil.followChoreoPath(
                    drive, state, Optional.ofNullable(choreoChooser.get()).orElse(""), red);
            case PATHFIND_TO_LOCATION -> AutoFactory.pathfindTo(
                    drive, state, () -> selected(targetChooser, FieldLocation.HUB_NEAR), red);
            case ADAPTIVE_ROUTE -> new AdaptiveAutoBuilder(drive, state, red)
                    .targets(
                            () -> selected(adaptiveFirstChooser, FieldLocation.DEPOT),
                            () -> selected(adaptiveSecondChooser, FieldLocation.HUB_NEAR),
                            () -> selected(adaptiveThirdChooser, FieldLocation.DEPOT))
                    .visitFirstTarget(true)
                    .visitSecondTarget(true)
                    .visitThirdTarget(true)
                    .build();
        };
    }

    private static FieldLocation selected(
            LoggedDashboardChooser<FieldLocation> chooser, FieldLocation fallback) {
        return Optional.ofNullable(chooser.get()).orElse(fallback);
    }

    private Command loadPathPlannerAuto(String name) {
        if (name == null || name.isBlank()) return Commands.none();
        try {
            return new PathPlannerAuto(name);
        } catch (RuntimeException exception) {
            DriverStation.reportError(
                    "Unable to build PathPlanner auto " + name, exception.getStackTrace());
            return Commands.none();
        }
    }

    public Optional<Pose2d> getStartingPose(BooleanSupplier isRed) {
        DesiredMode mode = Optional.ofNullable(modeChooser.get()).orElse(DesiredMode.DO_NOTHING);
        if (mode == DesiredMode.PATHPLANNER_AUTO) {
            String name = pathPlannerChooser.get();
            if (name != null && !name.isBlank())
                try {
                    return Optional.ofNullable(new PathPlannerAuto(name).getStartingPose())
                            .map(
                                    pose ->
                                            isRed.getAsBoolean()
                                                    ? FlippingUtil.flipFieldPose(pose)
                                                    : pose);
                } catch (RuntimeException ignored) {
                }
        }
        if (mode == DesiredMode.CHOREO_PATH)
            return AutoUtil.getChoreoStartingPose(
                    Optional.ofNullable(choreoChooser.get()).orElse(""), isRed.getAsBoolean());
        return Optional.empty();
    }

    public boolean haveSelectionsChanged() {
        boolean changed =
                modeChooser.get() != lastMode
                        || !Optional.ofNullable(pathPlannerChooser.get())
                                .orElse("")
                                .equals(lastPath)
                        || !Optional.ofNullable(choreoChooser.get()).orElse("").equals(lastChoreo)
                        || targetChooser.get() != lastTarget
                        || adaptiveFirstChooser.get() != lastAdaptiveFirst
                        || adaptiveSecondChooser.get() != lastAdaptiveSecond
                        || adaptiveThirdChooser.get() != lastAdaptiveThird
                        || !DriverStation.getAlliance().equals(lastAlliance);
        if (changed) {
            lastMode = modeChooser.get();
            lastPath = Optional.ofNullable(pathPlannerChooser.get()).orElse("");
            lastChoreo = Optional.ofNullable(choreoChooser.get()).orElse("");
            lastTarget = targetChooser.get();
            lastAdaptiveFirst = adaptiveFirstChooser.get();
            lastAdaptiveSecond = adaptiveSecondChooser.get();
            lastAdaptiveThird = adaptiveThirdChooser.get();
            lastAlliance = DriverStation.getAlliance();
        }
        return changed;
    }

    public void invalidateCache() {
        lastMode = null;
    }
}
