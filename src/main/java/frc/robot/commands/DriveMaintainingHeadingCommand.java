package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveMaintainingHeadingCommand extends Command {
    public DriveMaintainingHeadingCommand(
            DriveSubsystem drivetrain,
            RobotState robotState,
            DoubleSupplier throttle,
            DoubleSupplier strafe,
            DoubleSupplier turn) {
        mDrivetrain = drivetrain;
        mRobotState = robotState;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        driveWithHeading.HeadingController.setPID(
                DriveConstants.kHeadingControllerP,
                DriveConstants.kHeadingControllerI,
                DriveConstants.kHeadingControllerD);

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        // Note: Removed OpenLoopVoltage override for simulation.
        // Velocity control provides better braking behavior with maple-sim 0.4.0-beta.
    }

    private final RobotState mRobotState;
    protected DriveSubsystem mDrivetrain;
    private final DoubleSupplier mThrottleSupplier;
    private final DoubleSupplier mStrafeSupplier;
    private final DoubleSupplier mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            DriveConstants.kDriveMaxSpeed * 0.05) // Add a 5% deadband in open loop
                    .withRotationalDeadband(
                            DriveConstants.kDriveMaxAngularRate * Constants.kSteerJoystickDeadband)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(DriveConstants.kDriveMaxSpeed * 0.05)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = mRobotState.isRedAlliance() ? -strafe : strafe;
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        if (Math.abs(turnFieldFrame) > Constants.kSteerJoystickDeadband
                || (Util.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(
                                        mRobotState.getLatestRobotRelativeChassisSpeed()
                                                .omegaRadiansPerSecond)
                                > Math.toRadians(10))) {
            mDrivetrain.setControl(
                    (driveNoHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withRotationalRate(
                                    turnFieldFrame * DriveConstants.kDriveMaxAngularRate)));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint =
                        Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());
            }
            Logger.recordOutput("DriveMaintainHeading/throttleFieldFrame", throttleFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/strafeFieldFrame", strafeFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/mHeadingSetpoint", mHeadingSetpoint.get());
            mDrivetrain.setControl(
                    driveWithHeading
                            .withVelocityX(throttleFieldFrame)
                            .withVelocityY(strafeFieldFrame)
                            .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput(
                    "DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
