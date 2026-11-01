package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterAiming;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;

/** Stops translation, aims the chassis, and feeds only when the fixed shooter is on solution. */
public class AimAndShootCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final RobotState robotState;
    private final Superstructure superstructure;
    private final boolean allianceCornerFerry;

    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(DriveConstants.kDriveMaxSpeed * 0.05)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public AimAndShootCommand(
            DriveSubsystem driveSubsystem, RobotState robotState, Superstructure superstructure) {
        this(driveSubsystem, robotState, superstructure, false);
    }

    public AimAndShootCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Superstructure superstructure,
            boolean allianceCornerFerry) {
        this.driveSubsystem = driveSubsystem;
        this.robotState = robotState;
        this.superstructure = superstructure;
        this.allianceCornerFerry = allianceCornerFerry;

        driveWithHeading.HeadingController.setPID(
                DriveConstants.kHeadingControllerP,
                DriveConstants.kHeadingControllerI,
                DriveConstants.kHeadingControllerD);

        addRequirements(driveSubsystem, superstructure);
        setName(allianceCornerFerry ? "Alliance Corner Ferry" : "Aim and Shoot at Hub");
    }

    @Override
    public void initialize() {
        superstructure.setGoal(Goal.SHOOTING);
    }

    @Override
    public void execute() {
        ShooterAiming.ShotSolution solution =
                allianceCornerFerry
                        ? ShooterAiming.calculateAllianceCorner(robotState)
                        : ShooterAiming.calculate(robotState);
        superstructure.setShootingRequest(
                solution.shooterVelocityRadPerSec(),
                solution.hoodAngleRad(),
                solution.readyToFeed());

        driveSubsystem.setControl(
                driveWithHeading
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withTargetDirection(solution.robotHeading()));
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.clearShootingRequest();
        // Preserve the intake's deployed post-shot state while stopping the shooter and rollers.
        superstructure.setGoal(Goal.DEPLOYED_IDLE);
    }
}
