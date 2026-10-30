package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.lib.time.RobotTime;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Hardware implementation of the drivetrain IO interface using Phoenix 6 swerve. */
public class DriveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements DriveIO {

    private final AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

    private final double wheelRadiusMeters;

    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains sysIdSteerRequest =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation sysIdRotationRequest =
            new SwerveRequest.SysIdSwerveRotation();

    /** Closed-loop wheel speed request for feedforward and wheel-radius characterization. */
    private final SwerveRequest.ApplyRobotSpeeds applySpeedsRequest =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<Voltage>[] driveMotorVoltageSignals;
    private final StatusSignal<Voltage>[] steerMotorVoltageSignals;
    private final StatusSignal<AngularVelocity>[] moduleSteerVelocitySignals;
    private final StatusSignal<Angle>[] moduleAbsolutePositionSignals;
    private final double[] previousSteerAbsoluteRotations;
    private final double[] continuousSteerRotations;
    private final boolean[] haveSteerPosition;
    private final AtomicReference<double[]> latestSteerPositionsRad =
            new AtomicReference<>(new double[0]);
    private double previousYawDegrees;
    private double continuousYawRadians;
    private boolean haveYawPosition;

    private final RobotState robotState;
    final Consumer<SwerveDriveState> telemetryConsumer_;

    public DriveIOHardware(
            RobotState robotState,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, 100.0, modules);
        if (modules.length == 0 || modules[0].WheelRadius <= 0.0) {
            throw new IllegalArgumentException(
                    "At least one module with a valid wheel radius is required");
        }

        this.robotState = robotState;
        wheelRadiusMeters = modules[0].WheelRadius;
        telemetryConsumer_ =
                driveState -> {
                    telemetryCache.set(driveState.clone());
                    robotState.addOdometryMeasurement(
                            (RobotTime.getTimestampSeconds() - Utils.getCurrentTimeSeconds())
                                    + driveState.Timestamp,
                            driveState.Pose);
                };

        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        yaw = getPigeon2().getYaw();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        int moduleCount = getModules().length;
        @SuppressWarnings("unchecked")
        StatusSignal<Voltage>[] driveSignals =
                (StatusSignal<Voltage>[]) new StatusSignal<?>[moduleCount];
        @SuppressWarnings("unchecked")
        StatusSignal<Voltage>[] steerSignals =
                (StatusSignal<Voltage>[]) new StatusSignal<?>[moduleCount];
        @SuppressWarnings("unchecked")
        StatusSignal<AngularVelocity>[] steerVelocitySignals =
                (StatusSignal<AngularVelocity>[]) new StatusSignal<?>[moduleCount];
        @SuppressWarnings("unchecked")
        StatusSignal<Angle>[] absolutePositionSignals =
                (StatusSignal<Angle>[]) new StatusSignal<?>[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            driveSignals[i] = getModule(i).getDriveMotor().getMotorVoltage();
            steerSignals[i] = getModule(i).getSteerMotor().getMotorVoltage();
            // The CANcoder measures the module azimuth directly, so no motor gear-ratio
            // conversion is needed for module steering velocity.
            steerVelocitySignals[i] = getModule(i).getEncoder().getVelocity();
            absolutePositionSignals[i] = getModule(i).getEncoder().getAbsolutePosition();
        }
        driveMotorVoltageSignals = driveSignals;
        steerMotorVoltageSignals = steerSignals;
        moduleSteerVelocitySignals = steerVelocitySignals;
        moduleAbsolutePositionSignals = absolutePositionSignals;
        previousSteerAbsoluteRotations = new double[moduleCount];
        continuousSteerRotations = new double[moduleCount];
        haveSteerPosition = new boolean[moduleCount];

        BaseStatusSignal.setUpdateFrequencyForAll(100, angularYawVelocity, yaw);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                angularPitchVelocity,
                angularRollVelocity,
                roll,
                pitch,
                accelerationX,
                accelerationY);
        BaseStatusSignal.setUpdateFrequencyForAll(100, driveMotorVoltageSignals);
        BaseStatusSignal.setUpdateFrequencyForAll(100, steerMotorVoltageSignals);
        BaseStatusSignal.setUpdateFrequencyForAll(100, moduleSteerVelocitySignals);
        BaseStatusSignal.setUpdateFrequencyForAll(100, moduleAbsolutePositionSignals);

        getOdometryThread().setThreadPriority(99);
        registerTelemetry(telemetryConsumer_);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public Command applyRequest(
            Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired) {
        return Commands.run(() -> setControl(requestSupplier.get()), subsystemRequired);
    }

    @Override
    public void addVisionMeasurement(VisionFieldPoseEstimate visionEstimate) {
        double timestamp = Utils.fpgaToCurrentTime(visionEstimate.getTimestampSeconds());
        if (visionEstimate.getVisionMeasurementStdDevs() == null) {
            super.addVisionMeasurement(visionEstimate.getVisionRobotPoseMeters(), timestamp);
        } else {
            super.addVisionMeasurement(
                    visionEstimate.getVisionRobotPoseMeters(),
                    timestamp,
                    visionEstimate.getVisionMeasurementStdDevs());
        }
    }

    @Override
    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        super.setStateStdDevs(stateStdDevs);
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void setDriveOpenLoopVoltage(Voltage volts) {
        setControl(sysIdTranslationRequest.withVolts(volts));
    }

    @Override
    public void setDriveVelocity(ChassisSpeeds speeds) {
        setControl(applySpeedsRequest.withSpeeds(speeds));
    }

    @Override
    public void setSteerOpenLoopVoltage(Voltage volts) {
        setControl(sysIdSteerRequest.withVolts(volts));
    }

    @Override
    public void setRobotRotationRate(AngularVelocity rotationRate) {
        setControl(sysIdRotationRequest.withRotationalRate(rotationRate));
    }

    @Override
    public Rotation2d getRotation() {
        SwerveDriveState driveState = telemetryCache.get();
        return driveState != null ? driveState.Pose.getRotation() : new Rotation2d();
    }

    @Override
    public double[] getWheelRadiusCharacterizationPositions() {
        SwerveDriveState driveState = telemetryCache.get();
        if (driveState == null) {
            return new double[getModules().length];
        }

        SwerveModulePosition[] positions = driveState.ModulePositions;
        double[] wheelAnglesRadians = new double[positions.length];
        for (int i = 0; i < positions.length; i++) {
            // Phoenix module positions already include coupling compensation and are scaled by
            // the configured radius; dividing recovers cumulative drive-axle radians.
            wheelAnglesRadians[i] = positions[i].distanceMeters / wheelRadiusMeters;
        }
        return wheelAnglesRadians;
    }

    @Override
    public double[] getModuleSteerPositionsRad() {
        return latestSteerPositionsRad.get().clone();
    }

    @Override
    public double[] getModuleSteerVelocitiesRadPerSec() {
        BaseStatusSignal.refreshAll(moduleSteerVelocitySignals);
        int moduleCount = getModules().length;
        double[] velocities = new double[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            velocities[i] =
                    Units.rotationsToRadians(moduleSteerVelocitySignals[i].getValueAsDouble());
        }
        return velocities;
    }

    @Override
    public double getFFCharacterizationVelocity() {
        SwerveDriveState driveState = telemetryCache.get();
        if (driveState == null || driveState.ModuleStates.length == 0) {
            return 0.0;
        }

        double wheelVelocityRadiansPerSecond = 0.0;
        for (SwerveModuleState state : driveState.ModuleStates) {
            wheelVelocityRadiansPerSecond += state.speedMetersPerSecond / wheelRadiusMeters;
        }
        return wheelVelocityRadiansPerSecond / driveState.ModuleStates.length;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState driveState = telemetryCache.get();
        if (driveState == null) {
            return;
        }

        inputs.pose = driveState.Pose;
        inputs.successfulDaqs = driveState.SuccessfulDaqs;
        inputs.failedDaqs = driveState.FailedDaqs;
        inputs.moduleStates = driveState.ModuleStates;
        inputs.moduleTargets = driveState.ModuleTargets;
        inputs.modulePositions = driveState.ModulePositions;
        inputs.speeds = driveState.Speeds;
        inputs.odometryPeriod = driveState.OdometryPeriod;
        inputs.gyroYawDeg = driveState.Pose.getRotation().getDegrees();

        int moduleCount = getModules().length;
        if (inputs.moduleAbsolutePositions.length != moduleCount) {
            inputs.moduleAbsolutePositions = new Rotation2d[moduleCount];
        }
        if (inputs.moduleDriveAppliedVolts.length != moduleCount) {
            inputs.moduleDriveAppliedVolts = new double[moduleCount];
        }
        if (inputs.moduleSteerAppliedVolts.length != moduleCount) {
            inputs.moduleSteerAppliedVolts = new double[moduleCount];
        }
        if (inputs.moduleSteerPositionsRad.length != moduleCount) {
            inputs.moduleSteerPositionsRad = new double[moduleCount];
        }
        if (inputs.moduleSteerVelocitiesRadPerSec.length != moduleCount) {
            inputs.moduleSteerVelocitiesRadPerSec = new double[moduleCount];
        }

        BaseStatusSignal.refreshAll(driveMotorVoltageSignals);
        BaseStatusSignal.refreshAll(steerMotorVoltageSignals);
        BaseStatusSignal.refreshAll(moduleAbsolutePositionSignals);
        BaseStatusSignal.refreshAll(moduleSteerVelocitySignals);
        for (int i = 0; i < moduleCount; i++) {
            double absoluteRotations = moduleAbsolutePositionSignals[i].getValueAsDouble();
            if (!haveSteerPosition[i]) {
                continuousSteerRotations[i] = absoluteRotations;
                haveSteerPosition[i] = true;
            } else {
                continuousSteerRotations[i] +=
                        Math.IEEEremainder(
                                absoluteRotations - previousSteerAbsoluteRotations[i], 1.0);
            }
            previousSteerAbsoluteRotations[i] = absoluteRotations;
            inputs.moduleAbsolutePositions[i] = Rotation2d.fromRotations(absoluteRotations);
            inputs.moduleSteerPositionsRad[i] =
                    Units.rotationsToRadians(continuousSteerRotations[i]);
            inputs.moduleSteerVelocitiesRadPerSec[i] =
                    Units.rotationsToRadians(moduleSteerVelocitySignals[i].getValueAsDouble());
            inputs.moduleDriveAppliedVolts[i] = driveMotorVoltageSignals[i].getValueAsDouble();
            inputs.moduleSteerAppliedVolts[i] = steerMotorVoltageSignals[i].getValueAsDouble();
        }
        latestSteerPositionsRad.set(inputs.moduleSteerPositionsRad.clone());

        ChassisSpeeds measuredRobotRelativeSpeeds =
                getKinematics().toChassisSpeeds(inputs.moduleStates);
        ChassisSpeeds measuredFieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        measuredRobotRelativeSpeeds, inputs.pose.getRotation());
        ChassisSpeeds desiredRobotRelativeSpeeds =
                getKinematics().toChassisSpeeds(inputs.moduleTargets);
        ChassisSpeeds desiredFieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        desiredRobotRelativeSpeeds, inputs.pose.getRotation());

        BaseStatusSignal.refreshAll(
                angularRollVelocity,
                angularPitchVelocity,
                angularYawVelocity,
                pitch,
                roll,
                yaw,
                accelerationX,
                accelerationY);

        double timestamp = RobotTime.getTimestampSeconds();
        double rollRadiansPerSecond =
                Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
        double pitchRadiansPerSecond =
                Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
        double yawRadiansPerSecond = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        double yawDegrees = yaw.getValueAsDouble();
        if (!haveYawPosition) {
            continuousYawRadians = Units.degreesToRadians(yawDegrees);
            haveYawPosition = true;
        } else {
            continuousYawRadians +=
                    Units.degreesToRadians(
                            Math.IEEEremainder(yawDegrees - previousYawDegrees, 360.0));
        }
        previousYawDegrees = yawDegrees;
        inputs.gyroYawRad = continuousYawRadians;
        inputs.gyroYawRadPerSec = yawRadiansPerSecond;
        ChassisSpeeds fusedFieldRelativeSpeeds =
                new ChassisSpeeds(
                        measuredFieldRelativeSpeeds.vxMetersPerSecond,
                        measuredFieldRelativeSpeeds.vyMetersPerSecond,
                        yawRadiansPerSecond);

        robotState.addDriveMotionMeasurements(
                timestamp,
                rollRadiansPerSecond,
                pitchRadiansPerSecond,
                yawRadiansPerSecond,
                Units.degreesToRadians(pitch.getValueAsDouble()),
                Units.degreesToRadians(roll.getValueAsDouble()),
                accelerationX.getValueAsDouble(),
                accelerationY.getValueAsDouble(),
                desiredRobotRelativeSpeeds,
                desiredFieldRelativeSpeeds,
                measuredRobotRelativeSpeeds,
                measuredFieldRelativeSpeeds,
                fusedFieldRelativeSpeeds);
    }
}
