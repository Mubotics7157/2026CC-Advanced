package frc.robot.auto;

import com.team254.lib.pathplanner.commands.PathPlannerAuto;
import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.factories.AutoFactory;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Dashboard configuration for all Rebuilt file, Choreo, and adaptive autonomous routines. */
public class AutoModeSelector {
    private static final double AUTO_INTAKE_DWELL_SECONDS = 1.2;
    private static final double AUTO_SHOOT_DWELL_SECONDS = 4.0;

    public enum DesiredMode {
        DO_NOTHING,
        SYSID_QUASISTATIC_FORWARD,
        SYSID_QUASISTATIC_REVERSE,
        SYSID_DYNAMIC_FORWARD,
        SYSID_DYNAMIC_REVERSE,
        SYSID_STEER_QUASISTATIC_FORWARD,
        SYSID_STEER_QUASISTATIC_REVERSE,
        SYSID_STEER_DYNAMIC_FORWARD,
        SYSID_STEER_DYNAMIC_REVERSE,
        SYSID_ROTATION_QUASISTATIC_FORWARD,
        SYSID_ROTATION_QUASISTATIC_REVERSE,
        SYSID_ROTATION_DYNAMIC_FORWARD,
        SYSID_ROTATION_DYNAMIC_REVERSE,
        PATHPLANNER_AUTO,
        CHOREO_PATH,
        MODULAR_AUTO,
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
    private final LoggedDashboardChooser<ModularAutoBuilder.StartSide> modularStartChooser =
            new LoggedDashboardChooser<>("Auto/Modular/StartSide");
    private final LoggedDashboardChooser<ModularAutoBuilder.IntakeLocation>
            modularFirstIntakeChooser = new LoggedDashboardChooser<>("Auto/Modular/FirstIntake");
    private final LoggedDashboardChooser<ModularAutoBuilder.ShootPosition>
            modularFirstShootChooser = new LoggedDashboardChooser<>("Auto/Modular/FirstShoot");
    private final LoggedDashboardChooser<ModularAutoBuilder.IntakeLocation>
            modularSecondIntakeChooser = new LoggedDashboardChooser<>("Auto/Modular/SecondIntake");
    private final LoggedDashboardChooser<ModularAutoBuilder.ShootPosition>
            modularSecondShootChooser = new LoggedDashboardChooser<>("Auto/Modular/SecondShoot");
    private DesiredMode lastMode;
    private String lastPath;
    private String lastChoreo;
    private FieldLocation lastTarget;
    private FieldLocation lastAdaptiveFirst;
    private FieldLocation lastAdaptiveSecond;
    private FieldLocation lastAdaptiveThird;
    private ModularAutoBuilder.StartSide lastModularStart;
    private ModularAutoBuilder.IntakeLocation lastModularFirstIntake;
    private ModularAutoBuilder.ShootPosition lastModularFirstShoot;
    private ModularAutoBuilder.IntakeLocation lastModularSecondIntake;
    private ModularAutoBuilder.ShootPosition lastModularSecondShoot;
    private Optional<DriverStation.Alliance> lastAlliance = Optional.empty();

    public AutoModeSelector(RobotContainer container) {
        this.container = container;
        modeChooser.addDefaultOption("Modular Two Cycle Auto", DesiredMode.MODULAR_AUTO);
        modeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser.addOption(
                "Drive SysId (Quasistatic Forward)", DesiredMode.SYSID_QUASISTATIC_FORWARD);
        modeChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", DesiredMode.SYSID_QUASISTATIC_REVERSE);
        modeChooser.addOption("Drive SysId (Dynamic Forward)", DesiredMode.SYSID_DYNAMIC_FORWARD);
        modeChooser.addOption("Drive SysId (Dynamic Reverse)", DesiredMode.SYSID_DYNAMIC_REVERSE);
        modeChooser.addOption(
                "Steer SysId (Quasistatic Forward)", DesiredMode.SYSID_STEER_QUASISTATIC_FORWARD);
        modeChooser.addOption(
                "Steer SysId (Quasistatic Reverse)", DesiredMode.SYSID_STEER_QUASISTATIC_REVERSE);
        modeChooser.addOption(
                "Steer SysId (Dynamic Forward)", DesiredMode.SYSID_STEER_DYNAMIC_FORWARD);
        modeChooser.addOption(
                "Steer SysId (Dynamic Reverse)", DesiredMode.SYSID_STEER_DYNAMIC_REVERSE);
        modeChooser.addOption(
                "Rotation SysId (Quasistatic Forward)",
                DesiredMode.SYSID_ROTATION_QUASISTATIC_FORWARD);
        modeChooser.addOption(
                "Rotation SysId (Quasistatic Reverse)",
                DesiredMode.SYSID_ROTATION_QUASISTATIC_REVERSE);
        modeChooser.addOption(
                "Rotation SysId (Dynamic Forward)", DesiredMode.SYSID_ROTATION_DYNAMIC_FORWARD);
        modeChooser.addOption(
                "Rotation SysId (Dynamic Reverse)", DesiredMode.SYSID_ROTATION_DYNAMIC_REVERSE);
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
        configureModularChoosers();
    }

    private void configureModularChoosers() {
        modularStartChooser.addDefaultOption("Left", ModularAutoBuilder.StartSide.LEFT);
        modularStartChooser.addOption("Right", ModularAutoBuilder.StartSide.RIGHT);
        modularStartChooser.addOption("Middle", ModularAutoBuilder.StartSide.MIDDLE);

        modularFirstIntakeChooser.addDefaultOption(
                "Center", ModularAutoBuilder.IntakeLocation.CENTER);
        modularFirstIntakeChooser.addOption("Depot", ModularAutoBuilder.IntakeLocation.DEPOT);
        modularFirstIntakeChooser.addOption("Outpost", ModularAutoBuilder.IntakeLocation.OUTPOST);
        modularFirstIntakeChooser.addOption(
                "Left (Pathfind)", ModularAutoBuilder.IntakeLocation.LEFT);
        modularFirstIntakeChooser.addOption(
                "Right (Pathfind)", ModularAutoBuilder.IntakeLocation.RIGHT);
        modularFirstIntakeChooser.addOption("Preload", ModularAutoBuilder.IntakeLocation.PRELOAD);
        modularFirstIntakeChooser.addOption(
                "Do Nothing", ModularAutoBuilder.IntakeLocation.DO_NOTHING);

        modularFirstShootChooser.addDefaultOption(
                "Shoot In Place", ModularAutoBuilder.ShootPosition.IN_PLACE);
        modularFirstShootChooser.addOption("Near Hub", ModularAutoBuilder.ShootPosition.NEAR);
        modularFirstShootChooser.addOption("Far Hub", ModularAutoBuilder.ShootPosition.FAR);
        modularFirstShootChooser.addOption("Left Hub", ModularAutoBuilder.ShootPosition.LEFT);
        modularFirstShootChooser.addOption("Right Hub", ModularAutoBuilder.ShootPosition.RIGHT);

        modularSecondIntakeChooser.addDefaultOption(
                "Center", ModularAutoBuilder.IntakeLocation.CENTER);
        modularSecondIntakeChooser.addOption("Depot", ModularAutoBuilder.IntakeLocation.DEPOT);
        modularSecondIntakeChooser.addOption("Outpost", ModularAutoBuilder.IntakeLocation.OUTPOST);
        modularSecondIntakeChooser.addOption(
                "Left (Pathfind)", ModularAutoBuilder.IntakeLocation.LEFT);
        modularSecondIntakeChooser.addOption(
                "Right (Pathfind)", ModularAutoBuilder.IntakeLocation.RIGHT);
        modularSecondIntakeChooser.addOption(
                "Do Nothing", ModularAutoBuilder.IntakeLocation.DO_NOTHING);

        modularSecondShootChooser.addDefaultOption(
                "Shoot In Place", ModularAutoBuilder.ShootPosition.IN_PLACE);
        modularSecondShootChooser.addOption("Near Hub", ModularAutoBuilder.ShootPosition.NEAR);
        modularSecondShootChooser.addOption("Far Hub", ModularAutoBuilder.ShootPosition.FAR);
        modularSecondShootChooser.addOption("Left Hub", ModularAutoBuilder.ShootPosition.LEFT);
        modularSecondShootChooser.addOption("Right Hub", ModularAutoBuilder.ShootPosition.RIGHT);
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
        Command command =
                switch (mode) {
                    case DO_NOTHING -> Commands.none();
                    case SYSID_QUASISTATIC_FORWARD -> drive.sysIdQuasistatic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Drive SysId Quasistatic Forward");
                    case SYSID_QUASISTATIC_REVERSE -> drive.sysIdQuasistatic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Drive SysId Quasistatic Reverse");
                    case SYSID_DYNAMIC_FORWARD -> drive.sysIdDynamic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Drive SysId Dynamic Forward");
                    case SYSID_DYNAMIC_REVERSE -> drive.sysIdDynamic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Drive SysId Dynamic Reverse");
                    case SYSID_STEER_QUASISTATIC_FORWARD -> drive.sysIdSteerQuasistatic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Steer SysId Quasistatic Forward");
                    case SYSID_STEER_QUASISTATIC_REVERSE -> drive.sysIdSteerQuasistatic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Steer SysId Quasistatic Reverse");
                    case SYSID_STEER_DYNAMIC_FORWARD -> drive.sysIdSteerDynamic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Steer SysId Dynamic Forward");
                    case SYSID_STEER_DYNAMIC_REVERSE -> drive.sysIdSteerDynamic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Steer SysId Dynamic Reverse");
                    case SYSID_ROTATION_QUASISTATIC_FORWARD -> drive.sysIdRotationQuasistatic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Rotation SysId Quasistatic Forward");
                    case SYSID_ROTATION_QUASISTATIC_REVERSE -> drive.sysIdRotationQuasistatic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Rotation SysId Quasistatic Reverse");
                    case SYSID_ROTATION_DYNAMIC_FORWARD -> drive.sysIdRotationDynamic(
                                    SysIdRoutine.Direction.kForward)
                            .withName("Rotation SysId Dynamic Forward");
                    case SYSID_ROTATION_DYNAMIC_REVERSE -> drive.sysIdRotationDynamic(
                                    SysIdRoutine.Direction.kReverse)
                            .withName("Rotation SysId Dynamic Reverse");
                    case PATHPLANNER_AUTO -> loadPathPlannerAuto(pathPlannerChooser.get());
                    case CHOREO_PATH -> AutoUtil.followChoreoPath(
                            drive, state, Optional.ofNullable(choreoChooser.get()).orElse(""), red);
                    case MODULAR_AUTO -> buildModularAuto(red);
                    case PATHFIND_TO_LOCATION -> AutoFactory.pathfindTo(
                            drive,
                            state,
                            () -> selected(targetChooser, FieldLocation.HUB_NEAR),
                            red);
                    case ADAPTIVE_ROUTE -> buildAdaptiveRoute(drive, state, red);
                };
        if (mode == DesiredMode.PATHPLANNER_AUTO
                || mode == DesiredMode.CHOREO_PATH
                || mode == DesiredMode.PATHFIND_TO_LOCATION
                || mode == DesiredMode.ADAPTIVE_ROUTE) {
            Superstructure superstructure = container.getSuperstructure();
            return Commands.sequence(
                            Commands.runOnce(
                                    () -> superstructure.setGoal(Goal.DEPLOYED_IDLE),
                                    superstructure),
                            Commands.waitUntil(superstructure::atGoal).withTimeout(1.25),
                            command)
                    .withName("Deploy Intake Then " + command.getName());
        }
        return command;
    }

    private Command buildModularAuto(BooleanSupplier isRedAlliance) {
        ModularAutoBuilder.StartSide start =
                Optional.ofNullable(modularStartChooser.get())
                        .orElse(ModularAutoBuilder.StartSide.LEFT);
        ModularAutoBuilder.IntakeLocation first =
                Optional.ofNullable(modularFirstIntakeChooser.get())
                        .orElse(ModularAutoBuilder.IntakeLocation.CENTER);
        ModularAutoBuilder.IntakeLocation second =
                Optional.ofNullable(modularSecondIntakeChooser.get())
                        .orElse(ModularAutoBuilder.IntakeLocation.CENTER);
        ModularAutoBuilder.ShootPosition firstShoot =
                Optional.ofNullable(modularFirstShootChooser.get())
                        .orElse(ModularAutoBuilder.ShootPosition.IN_PLACE);
        ModularAutoBuilder.ShootPosition secondShoot =
                Optional.ofNullable(modularSecondShootChooser.get())
                        .orElse(ModularAutoBuilder.ShootPosition.IN_PLACE);

        Logger.recordOutput("Auto/Modular/StartSide", start.name());
        Logger.recordOutput("Auto/Modular/FirstIntake", first.name());
        Logger.recordOutput("Auto/Modular/FirstShoot", firstShoot.name());
        Logger.recordOutput("Auto/Modular/SecondIntake", second.name());
        Logger.recordOutput("Auto/Modular/SecondShoot", secondShoot.name());

        return new ModularAutoBuilder(
                        container.getDriveSubsystem(),
                        container.getRobotState(),
                        container.getSuperstructure(),
                        isRedAlliance)
                .startSide(start)
                .firstIntake(first)
                .firstShoot(firstShoot)
                .secondIntake(second)
                .secondShoot(secondShoot)
                .build();
    }

    private Command buildAdaptiveRoute(
            frc.robot.subsystems.drive.DriveSubsystem drive,
            frc.robot.RobotState state,
            BooleanSupplier isRedAlliance) {
        FieldLocation first = selected(adaptiveFirstChooser, FieldLocation.DEPOT);
        FieldLocation second = selected(adaptiveSecondChooser, FieldLocation.HUB_NEAR);
        FieldLocation third = selected(adaptiveThirdChooser, FieldLocation.DEPOT);
        Superstructure superstructure = container.getSuperstructure();

        return new AdaptiveAutoBuilder(drive, state, isRedAlliance)
                .targets(() -> first, () -> second, () -> third)
                .afterFirstTarget(autonomousIntakeAt(first, superstructure))
                .afterSecondTarget(autonomousShotAt(second, drive, state, superstructure))
                .afterThirdTarget(autonomousIntakeAt(third, superstructure))
                .visitFirstTarget(true)
                .visitSecondTarget(true)
                .visitThirdTarget(true)
                .build();
    }

    private static Command autonomousIntakeAt(FieldLocation target, Superstructure superstructure) {
        if (!target.isCollectionLocation()) {
            return Commands.none();
        }

        return Commands.run(() -> superstructure.setGoal(Goal.INTAKING), superstructure)
                .withTimeout(AUTO_INTAKE_DWELL_SECONDS)
                .finallyDo(() -> superstructure.setGoal(Goal.IDLE))
                .withName("Auto: Intake at " + target.name());
    }

    private static Command autonomousShotAt(
            FieldLocation target,
            frc.robot.subsystems.drive.DriveSubsystem drive,
            frc.robot.RobotState state,
            Superstructure superstructure) {
        if (!target.isHubStagingLocation()) {
            return Commands.none();
        }

        return new ShootOnTheMoveCommand(drive, state, superstructure, () -> 0.0, () -> 0.0)
                .withTimeout(AUTO_SHOOT_DWELL_SECONDS)
                .withName("Auto: Stationary Hub Shot");
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
        if (mode == DesiredMode.MODULAR_AUTO) {
            ModularAutoBuilder.StartSide start =
                    Optional.ofNullable(modularStartChooser.get())
                            .orElse(ModularAutoBuilder.StartSide.LEFT);
            return Optional.of(start.getPose(isRed.getAsBoolean()));
        }
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
                        || modularStartChooser.get() != lastModularStart
                        || modularFirstIntakeChooser.get() != lastModularFirstIntake
                        || modularFirstShootChooser.get() != lastModularFirstShoot
                        || modularSecondIntakeChooser.get() != lastModularSecondIntake
                        || modularSecondShootChooser.get() != lastModularSecondShoot
                        || !DriverStation.getAlliance().equals(lastAlliance);
        if (changed) {
            lastMode = modeChooser.get();
            lastPath = Optional.ofNullable(pathPlannerChooser.get()).orElse("");
            lastChoreo = Optional.ofNullable(choreoChooser.get()).orElse("");
            lastTarget = targetChooser.get();
            lastAdaptiveFirst = adaptiveFirstChooser.get();
            lastAdaptiveSecond = adaptiveSecondChooser.get();
            lastAdaptiveThird = adaptiveThirdChooser.get();
            lastModularStart = modularStartChooser.get();
            lastModularFirstIntake = modularFirstIntakeChooser.get();
            lastModularFirstShoot = modularFirstShootChooser.get();
            lastModularSecondIntake = modularSecondIntakeChooser.get();
            lastModularSecondShoot = modularSecondShootChooser.get();
            lastAlliance = DriverStation.getAlliance();
        }
        return changed;
    }

    public void invalidateCache() {
        lastMode = null;
    }
}
