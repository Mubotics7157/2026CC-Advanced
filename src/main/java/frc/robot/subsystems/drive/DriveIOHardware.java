package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.lib.time.RobotTime;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * The {@code DriveIOHardware} class implements the {@link DriveIO} interface using CTRE's
 * SwerveDrivetrain with TalonFX motors and CANcoders. It provides hardware-level control for the
 * swerve drive system.
 */
public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements DriveIO {

    private final AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;

    private final RobotState robotState;
    final Consumer<SwerveDriveState> telemetryConsumer_;

    public DriveIOHardware(
            RobotState robotState,
            SwerveDrivetrainConstants driveTrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, 250.0, modules);
        this.robotState = robotState;
        telemetryConsumer_ =
                swerveDriveState -> {
                    telemetryCache.set(swerveDriveState.clone());
                    this.robotState.addOdometryMeasurement(
                            (RobotTime.getTimestampSeconds() - Utils.getCurrentTimeSeconds())
                                    + swerveDriveState.Timestamp,
                            swerveDriveState.Pose);
                };

        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                angularPitchVelocity,
                angularRollVelocity,
                roll,
                pitch,
                accelerationX,
                accelerationY);

        this.getOdometryThread().setThreadPriority(99);

        registerTelemetry(telemetryConsumer_);
    }

    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemRequired);
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            this.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()));
        } else {
            this.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
                    visionFieldPoseEstimate.getVisionMeasurementStdDevs());
        }
    }

    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        this.setStateStdDevs(stateStdDevs);
    }

    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState driveState = telemetryCache.get();
        if (driveState == null) return;

        inputs.pose = driveState.Pose;
        inputs.successfulDaqs = driveState.SuccessfulDaqs;
        inputs.failedDaqs = driveState.FailedDaqs;
        inputs.moduleStates = driveState.ModuleStates;
        inputs.moduleTargets = driveState.ModuleTargets;
        inputs.speeds = driveState.Speeds;
        inputs.odometryPeriod = driveState.OdometryPeriod;

        Rotation2d gyroRotation = inputs.pose.getRotation();
        inputs.gyroYawDeg = gyroRotation.getDegrees();

        int moduleCount = getModules().length;
        if (inputs.moduleAbsolutePositions.length != moduleCount) {
            inputs.moduleAbsolutePositions = new Rotation2d[moduleCount];
        }
        for (int i = 0; i < moduleCount; i++) {
            inputs.moduleAbsolutePositions[i] =
                    Rotation2d.fromRotations(
                            getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble());
        }

        var measuredRobotRelativeChassisSpeeds =
                getKinematics().toChassisSpeeds(inputs.moduleStates);
        var measuredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        measuredRobotRelativeChassisSpeeds, gyroRotation);
        var desiredRobotRelativeChassisSpeeds =
                getKinematics().toChassisSpeeds(inputs.moduleTargets);
        var desiredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        desiredRobotRelativeChassisSpeeds, gyroRotation);

        BaseStatusSignal.refreshAll(
                angularRollVelocity,
                angularPitchVelocity,
                angularYawVelocity,
                pitch,
                roll,
                accelerationX,
                accelerationY);

        double timestamp = RobotTime.getTimestampSeconds();
        double rollRadsPerS = Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
        double pitchRadsPerS = Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
        double yawRadsPerS = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        // Trust gyro rate more than odometry.
        var fusedFieldRelativeChassisSpeeds =
                new ChassisSpeeds(
                        measuredFieldRelativeChassisSpeeds.vxMetersPerSecond,
                        measuredFieldRelativeChassisSpeeds.vyMetersPerSecond,
                        yawRadsPerS);

        double pitchRads = Units.degreesToRadians(pitch.getValueAsDouble());
        double rollRads = Units.degreesToRadians(roll.getValueAsDouble());
        double accelX = accelerationX.getValueAsDouble();
        double accelY = accelerationY.getValueAsDouble();
        robotState.addDriveMotionMeasurements(
                timestamp,
                rollRadsPerS,
                pitchRadsPerS,
                yawRadsPerS,
                pitchRads,
                rollRads,
                accelX,
                accelY,
                desiredRobotRelativeChassisSpeeds,
                desiredFieldRelativeChassisSpeeds,
                measuredRobotRelativeChassisSpeeds,
                measuredFieldRelativeChassisSpeeds,
                fusedFieldRelativeChassisSpeeds);
    }
}
