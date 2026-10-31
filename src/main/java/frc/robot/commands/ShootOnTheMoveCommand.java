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
import java.util.function.DoubleSupplier;

/** Spins up, aims the chassis, and feeds only when the fixed drum shooter is on solution. */
public class ShootOnTheMoveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final RobotState robotState;
    private final Superstructure superstructure;
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final boolean allianceCornerFerry;

    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(DriveConstants.kDriveMaxSpeed * 0.05)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public ShootOnTheMoveCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Superstructure superstructure,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier) {
        this(driveSubsystem, robotState, superstructure, throttleSupplier, strafeSupplier, false);
    }

    public ShootOnTheMoveCommand(
            DriveSubsystem driveSubsystem,
            RobotState robotState,
            Superstructure superstructure,
            DoubleSupplier throttleSupplier,
            DoubleSupplier strafeSupplier,
            boolean allianceCornerFerry) {
        this.driveSubsystem = driveSubsystem;
        this.robotState = robotState;
        this.superstructure = superstructure;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.allianceCornerFerry = allianceCornerFerry;

        driveWithHeading.HeadingController.setPID(
                DriveConstants.kHeadingControllerP,
                DriveConstants.kHeadingControllerI,
                DriveConstants.kHeadingControllerD);

        addRequirements(driveSubsystem, superstructure);
        setName(allianceCornerFerry ? "Alliance Corner Ferry" : "Shoot On The Move");
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

        double throttle = throttleSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double strafe = strafeSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double throttleFieldFrame = robotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = robotState.isRedAlliance() ? -strafe : strafe;

        driveSubsystem.setControl(
                driveWithHeading
                        .withVelocityX(throttleFieldFrame)
                        .withVelocityY(strafeFieldFrame)
                        .withTargetDirection(solution.robotHeading()));
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.clearShootingRequest();
        // Preserve the intake's deployed post-shot state while stopping the shooter and rollers.
        superstructure.setGoal(Goal.DEPLOYED_IDLE);
    }
}
