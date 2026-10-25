// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.team254.lib.pathplanner.commands.PathfindingCommand;
import com.team254.lib.pathplanner.pathfinding.Pathfinding;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final int kRTPriority = 2;
    private static final int kNonRTPriority = 1;

    private final RobotContainer robotContainer;
    private Command autonomousCommand = Commands.none();
    private Command disabledCommand = Commands.none();

    private int loopCount = 0;
    private int disabledLoopCounter = 0;
    private boolean autonomousCommandPrepared = false;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            if (!DriverStation.isFMSAttached()) {
                Logger.addDataReceiver(new NT4Publisher());
            }
        } else if (Constants.kIsReplay) {
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        } else if (RobotBase.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.start();
        if (!Logger.hasReplaySource()) {
            RobotController.setTimeSource(RobotController::getFPGATime);
        }

        robotContainer = new RobotContainer();

        if (RobotBase.isSimulation()) {
            robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        }

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SignalLogger.enableAutoLogging(false);

        // Warm PathPlanner pathfinding.
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        if (DriverStation.isEnabled()) {
            Threads.setCurrentThreadPriority(true, kRTPriority);
        } else {
            Threads.setCurrentThreadPriority(false, kNonRTPriority);
        }

        CommandScheduler.getInstance().run();
        robotContainer.getRobotState().updateLogger();

        // Periodic NetworkTables flush for sim/replay convenience.
        loopCount++;
        if (loopCount % 50 == 0) {
            NetworkTableInstance.getDefault().flush();
        }

        Threads.setCurrentThreadPriority(false, kNonRTPriority);
    }

    @Override
    public void disabledInit() {
        disabledCommand = robotContainer.getDisabledCommand();
        CommandScheduler.getInstance().schedule(disabledCommand);

        Pathfinding.ensureInitialized();
        Pathfinding.setTeleopObstacles();
        Pathfinding.enableCaching();
        Pathfinding.setCacheDistanceToleranceMeters(0.0);

        robotContainer
                .getDriveSubsystem()
                .getController()
                .accept(PathPlannerTrajectory.makeStayInPlaceTrajectory());
        robotContainer.getAutoModeSelector().invalidateCache();
        disabledLoopCounter = 50;
        autonomousCommandPrepared = false;
    }

    @Override
    public void disabledPeriodic() {
        // File parsing and path construction are done while disabled, never as autonomous begins.
        if (++disabledLoopCounter < 50) return;
        disabledLoopCounter = 0;
        if (!robotContainer.getAutoModeSelector().haveSelectionsChanged()) return;
        prepareAutonomousCommand();
    }

    private void prepareAutonomousCommand() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        autonomousCommandPrepared = true;
        Logger.recordOutput("Auto/RegeneratedCommand", autonomousCommand.getName());
        robotContainer
                .getAutoModeSelector()
                .getStartingPose(robotContainer.getRobotState()::isRedAlliance)
                .ifPresent(pose -> Logger.recordOutput("Auto/StartingPose", pose));
    }

    @Override
    public void autonomousInit() {
        disabledCommand.cancel();

        // The custom Local AD* planner uses a different obstacle model in autonomous than in
        // teleop. Select it before any adaptive or PathPlanner pathfinding command starts.
        Pathfinding.ensureInitialized();
        Pathfinding.setAutoObstacles();

        robotContainer.getRobotState().setAutoStartTime(Timer.getFPGATimestamp());

        if (!autonomousCommandPrepared
                || robotContainer.getAutoModeSelector().haveSelectionsChanged()) {
            prepareAutonomousCommand();
        }

        Optional<Pose2d> startingPose =
                robotContainer
                        .getAutoModeSelector()
                        .getStartingPose(() -> robotContainer.getRobotState().isRedAlliance());
        startingPose.ifPresent(robotContainer.getDriveSubsystem()::resetOdometry);

        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void teleopInit() {
        disabledCommand.cancel();
        autonomousCommand.cancel();
        Pathfinding.ensureInitialized();
        Pathfinding.setTeleopObstacles();
        // Revert any forced dumb mode so teleop starts with normal tolerance gating.
        robotContainer.getSuperstructure().setDumbModeEnabled(false);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
