package frc.robot.auto;

import com.team254.lib.pathplanner.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.factories.AutoFactory;
import frc.robot.lib.reefscape.ReefBranch;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING,
        PATHFIND_TO_FEEDER,
        PATHFIND_TO_BARGE,
        PATHFIND_TO_REEF_BRANCH
    }

    public enum StartingPosition {
        LEFT("Left Side", new Pose2d(7.0, 7.0, Rotation2d.fromDegrees(225.0))),
        MIDDLE("Middle Side", new Pose2d(7.2, 4.0, Rotation2d.fromDegrees(180.0))),
        RIGHT("Right Side", new Pose2d(7.0, 1.0, Rotation2d.fromDegrees(135.0)));

        private final String displayName;
        private final Pose2d startingPose;

        StartingPosition(String displayName, Pose2d startingPose) {
            this.displayName = displayName;
            this.startingPose = startingPose;
        }

        public String getDisplayName() {
            return displayName;
        }

        public Pose2d getStartingPose() {
            return startingPose;
        }
    }

    private final RobotContainer container;

    private final LoggedDashboardChooser<DesiredMode> modeChooser =
            new LoggedDashboardChooser<>("Auto/Mode");
    private final LoggedDashboardChooser<StartingPosition> startingPositionChooser =
            new LoggedDashboardChooser<>("Auto/StartingPosition");
    private final LoggedDashboardChooser<ReefBranch> reefBranchChooser =
            new LoggedDashboardChooser<>("Auto/ReefBranch");
    private final LoggedDashboardChooser<Boolean> useFirstLetterChooser =
            new LoggedDashboardChooser<>("Auto/UseFirstLetter");

    public AutoModeSelector(RobotContainer container) {
        this.container = container;

        modeChooser.addDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser.addOption("Pathfind: Feeder", DesiredMode.PATHFIND_TO_FEEDER);
        modeChooser.addOption("Pathfind: Barge", DesiredMode.PATHFIND_TO_BARGE);
        modeChooser.addOption("Pathfind: Reef Branch", DesiredMode.PATHFIND_TO_REEF_BRANCH);

        startingPositionChooser.addDefaultOption(
                StartingPosition.LEFT.getDisplayName(), StartingPosition.LEFT);
        startingPositionChooser.addOption(
                StartingPosition.MIDDLE.getDisplayName(), StartingPosition.MIDDLE);
        startingPositionChooser.addOption(
                StartingPosition.RIGHT.getDisplayName(), StartingPosition.RIGHT);

        reefBranchChooser.addDefaultOption("AB", ReefBranch.AB);
        reefBranchChooser.addOption("CD", ReefBranch.CD);
        reefBranchChooser.addOption("EF", ReefBranch.EF);
        reefBranchChooser.addOption("GH", ReefBranch.GH);
        reefBranchChooser.addOption("IJ", ReefBranch.IJ);
        reefBranchChooser.addOption("KL", ReefBranch.KL);

        useFirstLetterChooser.addDefaultOption("Left/First", true);
        useFirstLetterChooser.addOption("Right/Second", false);
    }

    public LoggedDashboardChooser<DesiredMode> getModeChooser() {
        return modeChooser;
    }

    public LoggedDashboardChooser<StartingPosition> getStartingPositionChooser() {
        return startingPositionChooser;
    }

    public Optional<Pose2d> getStartingPose(BooleanSupplier isRedAlliance) {
        StartingPosition selected = startingPositionChooser.get();
        if (selected == null) return Optional.empty();
        Pose2d pose = selected.getStartingPose();
        return Optional.of(isRedAlliance.getAsBoolean() ? FlippingUtil.flipFieldPose(pose) : pose);
    }

    public Command getAutonomousCommand() {
        DesiredMode selectedMode = modeChooser.get();
        if (selectedMode == null) {
            return Commands.none();
        }

        BooleanSupplier isRedAlliance = () -> container.getRobotState().isRedAlliance();
        var drive = container.getDriveSubsystem();
        var robotState = container.getRobotState();

        boolean useFirstLetter = Optional.ofNullable(useFirstLetterChooser.get()).orElse(false);

        return switch (selectedMode) {
            case DO_NOTHING -> Commands.none();
            case PATHFIND_TO_FEEDER -> AutoFactory.getPathfindToFeederCommand(
                    drive, robotState, () -> useFirstLetter, isRedAlliance);
            case PATHFIND_TO_BARGE -> AutoFactory.getPathfindToBargeCommand(
                    drive, robotState, isRedAlliance, null);
            case PATHFIND_TO_REEF_BRANCH -> AutoFactory.getPathfindToReefCommand(
                    drive,
                    robotState,
                    () -> Optional.ofNullable(reefBranchChooser.get()).orElse(ReefBranch.AB),
                    () -> useFirstLetter,
                    () -> 0.0,
                    Commands::none,
                    () -> true,
                    isRedAlliance,
                    false,
                    false);
        };
    }
}
