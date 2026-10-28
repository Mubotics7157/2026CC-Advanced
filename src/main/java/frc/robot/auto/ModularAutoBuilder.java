package frc.robot.auto;

import com.team254.lib.pathplanner.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.PathfindingAutoAlignCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.factories.AutoFactory;
import frc.robot.lib.rebuilt.FieldLocation;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Builds a selectable, two-cycle autonomous routine from Choreo paths and pathfinding legs. */
public final class ModularAutoBuilder {
    private static final double DEPLOY_TIMEOUT_SECONDS = 1.25;
    // Tune this dwell on the robot after testing the full cycle.
    private static final double SHOOT_SECONDS = 4.5;

    public enum StartSide {
        LEFT("Left", "LEFT", FieldLocation.LEFT),
        RIGHT("Right", "RIGHT", FieldLocation.RIGHT),
        MIDDLE("Middle", "MIDDLE", FieldLocation.DEPOT);

        private final String displayName;
        private final String pathSuffix;
        private final FieldLocation fallbackLocation;

        StartSide(String displayName, String pathSuffix, FieldLocation fallbackLocation) {
            this.displayName = displayName;
            this.pathSuffix = pathSuffix;
            this.fallbackLocation = fallbackLocation;
        }

        public String getDisplayName() {
            return displayName;
        }

        public String getRushPathName() {
            return "CenterRush_" + pathSuffix;
        }

        public String getRush2PathName() {
            return "CenterRush2_" + pathSuffix;
        }

        public String getReturnPathName() {
            return "CenterReturn_" + pathSuffix;
        }

        public String getReturn2PathName() {
            return "CenterReturn2_" + pathSuffix;
        }

        public String getDepotPathName() {
            return "DepotRush_" + pathSuffix;
        }

        public String getDepotReturnPathName() {
            return "DepotRushReturn_" + pathSuffix;
        }

        public String getPreloadPathName() {
            return "PreloadRush_" + pathSuffix;
        }

        public Pose2d getPose(boolean redAlliance) {
            // Routes from one start side share a branching pose. Use the first available
            // trajectory's starting pose so the branch target matches the authored paths.
            for (String pathName :
                    new String[] {
                        getRushPathName(),
                        getRush2PathName(),
                        getPreloadPathName(),
                        getDepotPathName()
                    }) {
                Optional<Pose2d> pathPose = AutoUtil.getChoreoStartingPose(pathName, redAlliance);
                if (pathPose.isPresent()) return pathPose.get();
            }
            return fallbackLocation.getPose(redAlliance);
        }
    }

    public enum IntakeLocation {
        CENTER("Center"),
        DEPOT("Depot"),
        OUTPOST("Outpost"),
        LEFT("Left"),
        RIGHT("Right"),
        PRELOAD("Preload"),
        DO_NOTHING("Do Nothing");

        private final String displayName;

        IntakeLocation(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    public enum ShootPosition {
        IN_PLACE("Shoot In Place", null),
        NEAR("Near Hub", FieldLocation.HUB_NEAR),
        FAR("Far Hub", FieldLocation.HUB_FAR),
        LEFT("Left Hub", FieldLocation.HUB_LEFT),
        RIGHT("Right Hub", FieldLocation.HUB_RIGHT);

        private final String displayName;
        private final FieldLocation location;

        ShootPosition(String displayName, FieldLocation location) {
            this.displayName = displayName;
            this.location = location;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    private final DriveSubsystem drive;
    private final RobotState robotState;
    private final Superstructure superstructure;
    private final BooleanSupplier isRedAlliance;
    private StartSide startSide = StartSide.LEFT;
    private IntakeLocation firstIntake = IntakeLocation.CENTER;
    private ShootPosition firstShoot = ShootPosition.IN_PLACE;
    private IntakeLocation secondIntake = IntakeLocation.CENTER;
    private ShootPosition secondShoot = ShootPosition.IN_PLACE;

    public ModularAutoBuilder(
            DriveSubsystem drive,
            RobotState robotState,
            Superstructure superstructure,
            BooleanSupplier isRedAlliance) {
        this.drive = drive;
        this.robotState = robotState;
        this.superstructure = superstructure;
        this.isRedAlliance = isRedAlliance;
    }

    public ModularAutoBuilder startSide(StartSide value) {
        startSide = value;
        return this;
    }

    public ModularAutoBuilder firstIntake(IntakeLocation value) {
        firstIntake = value;
        return this;
    }

    public ModularAutoBuilder firstShoot(ShootPosition value) {
        firstShoot = value;
        return this;
    }

    public ModularAutoBuilder secondIntake(IntakeLocation value) {
        secondIntake = value;
        return this;
    }

    public ModularAutoBuilder secondShoot(ShootPosition value) {
        secondShoot = value;
        return this;
    }

    public Command build() {
        Command auto =
                Commands.sequence(
                        logState("STARTING"),
                        Commands.runOnce(
                                () -> {
                                    Pose2d pose = startSide.getPose(isRedAlliance.getAsBoolean());
                                    drive.resetOdometry(pose);
                                    Logger.recordOutput("ModularAuto/StartPose", pose);
                                    superstructure.setGoal(Goal.DEPLOYED_IDLE);
                                }),
                        Commands.waitUntil(superstructure::atGoal)
                                .withTimeout(DEPLOY_TIMEOUT_SECONDS));
        boolean middleDepotAuto =
                startSide == StartSide.MIDDLE && firstIntake == IntakeLocation.DEPOT;

        if (firstIntake != IntakeLocation.DO_NOTHING) {
            auto =
                    auto.andThen(
                            logState("CYCLE1_PATHS"),
                            buildIntakeCycle(firstIntake, false),
                            logState("CYCLE1_SHOOT"),
                            middleDepotAuto ? buildShootForever() : buildShoot(firstShoot));
            if (!middleDepotAuto) {
                auto = auto.andThen(logState("CYCLE1_RETURN_TO_BRANCH"), buildBranchReset());
            }
        }

        if (!middleDepotAuto && secondIntake != IntakeLocation.DO_NOTHING) {
            auto =
                    auto.andThen(
                            logState("CYCLE2_PATHS"),
                            buildIntakeCycle(secondIntake, true),
                            logState("CYCLE2_SHOOT"),
                            buildShoot(secondShoot),
                            logState("CYCLE2_RETURN_TO_BRANCH"),
                            buildBranchReset());
        }

        if (middleDepotAuto) return auto.withName("Modular Middle Depot Auto");
        return auto.andThen(
                        logState("COMPLETED"),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.DEPLOYED_IDLE)))
                .withName("Modular Two Cycle Auto");
    }

    private Command buildIntakeCycle(IntakeLocation location, boolean secondCycle) {
        return switch (location) {
            case CENTER -> buildChoreoPair(
                    secondCycle ? startSide.getRush2PathName() : startSide.getRushPathName(),
                    secondCycle ? startSide.getReturn2PathName() : startSide.getReturnPathName());
            case DEPOT -> buildChoreoPair(
                    startSide.getDepotPathName(), startSide.getDepotReturnPathName());
            case OUTPOST -> buildPathfindIntake(FieldLocation.OUTPOST);
            case LEFT -> buildPathfindIntake(FieldLocation.LEFT);
            case RIGHT -> buildPathfindIntake(FieldLocation.RIGHT);
            case PRELOAD -> Commands.sequence(
                    followOrLogMissing(startSide.getPreloadPathName(), "PRELOAD"),
                    buildBranchReset());
            case DO_NOTHING -> Commands.none();
        };
    }

    private Command buildChoreoPair(String outbound, String returning) {
        if (!trajectoryExists(outbound)) return logMissing(outbound, "RUSH");
        Command outboundCommand = followOrLogMissing(outbound, "RUSH");
        Command returnCommand =
                trajectoryExists(returning)
                        ? AutoUtil.followChoreoPath(drive, robotState, returning, isRedAlliance)
                        : buildBranchReset();
        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    Logger.recordOutput("ModularAuto/RushPath", outbound);
                                    Logger.recordOutput(
                                            "ModularAuto/ReturnPath",
                                            trajectoryExists(returning)
                                                    ? returning
                                                    : "ADSTAR_TO_BRANCH");
                                }),
                        Commands.deadline(
                                outboundCommand,
                                superstructure.run(() -> superstructure.setGoal(Goal.INTAKING))),
                        returnCommand)
                .withName("Choreo Intake And Return: " + outbound);
    }

    private Command followOrLogMissing(String pathName, String pathType) {
        if (!trajectoryExists(pathName)) return logMissing(pathName, pathType);
        return AutoUtil.followChoreoPath(drive, robotState, pathName, isRedAlliance);
    }

    private static Command logMissing(String pathName, String pathType) {
        return Commands.runOnce(
                () -> {
                    Logger.recordOutput(
                            "ModularAuto/Warning", "MISSING_" + pathType + ": " + pathName);
                    DriverStation.reportWarning("Missing auto trajectory " + pathName, false);
                });
    }

    private Command buildPathfindIntake(FieldLocation location) {
        Command driveCommand =
                PathfindingAutoAlignCommand.create(drive, () -> location, isRedAlliance);
        return Commands.sequence(
                        Commands.deadline(
                                driveCommand,
                                superstructure.run(() -> superstructure.setGoal(Goal.INTAKING))),
                        buildBranchReset())
                .withName("Pathfind Intake: " + location.name());
    }

    private Command buildBranchReset() {
        return Commands.defer(
                        () -> {
                            Pose2d target = startSide.getPose(isRedAlliance.getAsBoolean());
                            Logger.recordOutput("ModularAuto/BranchTargetPose", target);
                            return AutoBuilder.pathfindToPose(
                                    target, PathfindingAutoAlignCommand.pathfindConstraints);
                        },
                        java.util.Set.of(drive))
                .withName("Pathfind Back To Branch");
    }

    private Command buildShoot(ShootPosition position) {
        Command moveToShot =
                position.location == null
                        ? Commands.none()
                        : AutoFactory.pathfindTo(
                                drive, robotState, () -> position.location, isRedAlliance);
        return Commands.sequence(
                        Commands.runOnce(() -> superstructure.setGoal(Goal.SHOOTING)),
                        moveToShot,
                        new ShootOnTheMoveCommand(
                                        drive, robotState, superstructure, () -> 0.0, () -> 0.0)
                                .withTimeout(SHOOT_SECONDS),
                        Commands.runOnce(() -> superstructure.setGoal(Goal.DEPLOYED_IDLE)))
                .withName("Shoot: " + position.name());
    }

    private Command buildShootForever() {
        return new ShootOnTheMoveCommand(drive, robotState, superstructure, () -> 0.0, () -> 0.0)
                .withName("Shoot In Place Until Auto Ends");
    }

    private static Command logState(String state) {
        return Commands.runOnce(
                () -> {
                    Logger.recordOutput("ModularAuto/State", state);
                    Logger.recordOutput("ModularAuto/Time", Timer.getFPGATimestamp());
                });
    }

    public Pose2d getStartingPose(boolean redAlliance) {
        return startSide.getPose(redAlliance);
    }

    public static boolean trajectoryExists(String trajectoryName) {
        return new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName + ".traj")
                .isFile();
    }
}
