// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoModeSelector;
import frc.robot.commands.DriveMaintainingHeadingCommand;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.factories.IntakeFactory;
import frc.robot.lib.util.MathHelpers;
import frc.robot.simulation.SimulatedRobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionIOHardwareLimelight;
import frc.robot.subsystems.vision.VisionIOSimPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class RobotContainer {
    private final AtomicReference<DriveSubsystem> driveForVision = new AtomicReference<>(null);
    private final ControlBoard controlBoard = ControlBoard.getInstance();
    private final SwerveDriveBrake xWheels = new SwerveDriveBrake();

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
            estimate -> {
                DriveSubsystem drive = driveForVision.get();
                if (drive != null) {
                    drive.addVisionMeasurement(estimate);
                }
            };

    private final RobotState robotState = new RobotState(visionEstimateConsumer);
    private final SimulatedRobotState simulatedRobotState =
            RobotBase.isSimulation() ? new SimulatedRobotState() : null;

    private final DriveSubsystem driveSubsystem = buildDriveSystem();
    // private final VisionSubsystem visionSubsystem = buildVisionSystem();
    // private final Intake intake = buildIntake();
    // private final Indexer indexer = buildIndexer();
    // private final Shooter shooter = buildShooter();
    // private final Superstructure superstructure = new Superstructure(intake, indexer, shooter);

    private final DriveMaintainingHeadingCommand driveCommand =
            new DriveMaintainingHeadingCommand(
                    driveSubsystem,
                    robotState,
                    controlBoard::getThrottle,
                    controlBoard::getStrafe,
                    controlBoard::getRotation);

    private final AutoModeSelector autoModeSelector = new AutoModeSelector(this);

    public RobotContainer() {
        SuperstructureConstants.init();
        driveForVision.set(driveSubsystem);
        configureBindings();
        SmartDashboard.putBoolean("Is Practice Bot", Constants.kIsPracticeBot);
    }

    private DriveSubsystem buildDriveSystem() {
        if (RobotBase.isSimulation()) {
            return new DriveSubsystem(
                    new DriveIOSim(
                            robotState,
                            simulatedRobotState,
                            DriveConstants.kDrivetrain.getDriveTrainConstants(),
                            DriveConstants.kDrivetrain.getModuleConstants()),
                    robotState);
        }
        return new DriveSubsystem(
                new DriveIOHardware(
                        robotState,
                        DriveConstants.kDrivetrain.getDriveTrainConstants(),
                        DriveConstants.kDrivetrain.getModuleConstants()),
                robotState);
    }

    private VisionSubsystem buildVisionSystem() {
        if (RobotBase.isSimulation()) {
            return new VisionSubsystem(
                    new VisionIOSimPhoton(robotState, simulatedRobotState), robotState);
        }
        return new VisionSubsystem(new VisionIOHardwareLimelight(robotState), robotState);
    }

    private Intake buildIntake() {
        Intake intake;
        if (RobotBase.isSimulation()) {
            intake = new Intake(new IntakeIOSim());
        } else {
            intake = new Intake(new IntakeIOReal());
        }
        intake.resetArmEncoderToDefault();
        return intake;
    }

    private Indexer buildIndexer() {
        if (RobotBase.isSimulation()) {
            return new Indexer(new IndexerIOSim());
        }
        return new Indexer(new IndexerIOReal());
    }

    private Shooter buildShooter() {
        if (RobotBase.isSimulation()) {
            return new Shooter(new ShooterIOSim());
        }
        return new Shooter(new ShooterIOReal());
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(driveCommand);
        controlBoard.resetGyro().onTrue(Commands.runOnce(this::resetHeading));
        controlBoard.getWantToXWheels().whileTrue(driveSubsystem.applyRequest(() -> xWheels));
        // controlBoard.getWantIntake().whileTrue(IntakeFactory.setIntakingCommand(superstructure));
        // controlBoard.getWantOuttake().whileTrue(IntakeFactory.setOuttakingCommand(superstructure));
        // controlBoard
        //         .getWantShoot()
        //         .whileTrue(
        //                 new ShootOnTheMoveCommand(
        //                         driveSubsystem,
        //                         robotState,
        //                         superstructure,
        //                         controlBoard::getThrottle,
        //                         controlBoard::getStrafe));
    }

    public void resetHeading() {
        driveSubsystem.resetOdometry(
                new Pose2d(
                        new Translation2d(
                                robotState.getLatestFieldToRobot().getValue().getX(),
                                robotState.getLatestFieldToRobot().getValue().getY()),
                        robotState.isRedAlliance()
                                ? MathHelpers.kRotation2dPi
                                : MathHelpers.kRotation2dZero));
    }

    public ControlBoard getControlBoard() {
        return controlBoard;
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    // public VisionSubsystem getVisionSubsystem() {
    //     return visionSubsystem;
    // }

    // public Intake getIntake() {
    //     return intake;
    // }

    // public Indexer getIndexer() {
    //     return indexer;
    // }

    // public Shooter getShooter() {
    //     return shooter;
    // }

    // public Superstructure getSuperstructure() {
    //     return superstructure;
    // }

    public RobotState getRobotState() {
        return robotState;
    }

    public SimulatedRobotState getSimulatedRobotState() {
        return simulatedRobotState;
    }

    public AutoModeSelector getAutoModeSelector() {
        return autoModeSelector;
    }

    public Command getAutonomousCommand() {
        return autoModeSelector.getAutonomousCommand();
    }

    public Command getDisabledCommand() {
        return Commands.none();
    }
}
