package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final SlewRateLimiter turnLimiter =
            new SlewRateLimiter(
                    DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared
                            / DriveConstants.kDriveMaxAngularRate);
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;
    private double lastTranslationX;
    private double lastTranslationY;
    private double lastTranslationUpdateTime;

    private final SwerveRequest.FieldCentric driveNoHeading =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriveConstants.kDriveMaxSpeed * 0.05)
                    .withRotationalDeadband(
                            DriveConstants.kDriveMaxAngularRate * Constants.kSteerJoystickDeadband)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(DriveConstants.kDriveMaxSpeed * 0.05)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint =
                Optional.of(mRobotState.getLatestFieldToRobot().getValue().getRotation());
        mJoystickLastTouched = -1.0;
        var measuredFieldSpeeds = mRobotState.getLatestMeasuredFieldRelativeChassisSpeeds();
        lastTranslationX = measuredFieldSpeeds.vxMetersPerSecond;
        lastTranslationY = measuredFieldSpeeds.vyMetersPerSecond;
        lastTranslationUpdateTime = Timer.getFPGATimestamp();
        turnLimiter.reset(
                MathUtil.clamp(
                        measuredFieldSpeeds.omegaRadiansPerSecond
                                / DriveConstants.kDriveMaxAngularRate,
                        -1.0,
                        1.0));
    }

    /** Limits the combined translation vector to the configured acceleration. */
    private void limitTranslationAcceleration(double requestedX, double requestedY) {
        double now = Timer.getFPGATimestamp();
        double elapsed = Math.max(0.0, Math.min(now - lastTranslationUpdateTime, 0.05));
        lastTranslationUpdateTime = now;

        double deltaX = requestedX - lastTranslationX;
        double deltaY = requestedY - lastTranslationY;
        double deltaMagnitude = Math.hypot(deltaX, deltaY);
        double maxDelta = DriveConstants.kTeleopMaxAccelerationMetersPerSecondSquared * elapsed;
        if (deltaMagnitude > maxDelta && deltaMagnitude > 0.0) {
            double scale = maxDelta / deltaMagnitude;
            deltaX *= scale;
            deltaY *= scale;
        }

        lastTranslationX += deltaX;
        lastTranslationY += deltaY;
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double strafe = mStrafeSupplier.getAsDouble() * DriveConstants.kDriveMaxSpeed;
        double turnFieldFrame = turnLimiter.calculate(mTurnSupplier.getAsDouble());
        double throttleFieldFrame = mRobotState.isRedAlliance() ? -throttle : throttle;
        double strafeFieldFrame = mRobotState.isRedAlliance() ? -strafe : strafe;
        limitTranslationAcceleration(throttleFieldFrame, strafeFieldFrame);
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
                            .withVelocityX(lastTranslationX)
                            .withVelocityY(lastTranslationY)
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
                            .withVelocityX(lastTranslationX)
                            .withVelocityY(lastTranslationY)
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
