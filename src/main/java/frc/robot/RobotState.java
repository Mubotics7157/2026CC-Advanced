package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.util.ConcurrentTimeInterpolatableBuffer;
import frc.robot.lib.util.FieldConstants;
import frc.robot.lib.util.MathHelpers;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Tracks robot state including pose, velocities, and vision measurements. */
public class RobotState {
    public static final double LOOKBACK_TIME = 1.0;

    private final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer;

    public RobotState(Consumer<VisionFieldPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }

    // Robot pose in field coordinates over time.
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);

    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private double lastUsedMegatagTimestamp = 0.0;
    private Pose2d lastUsedMegatagPose = Pose2d.kZero;

    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private double autoStartTime = 0.0;

    private Optional<Pose2d> trajectoryTargetPose = Optional.empty();
    private Optional<Pose2d> trajectoryCurrentPose = Optional.empty();

    private final AtomicReference<Optional<Integer>> exclusiveTag =
            new AtomicReference<>(Optional.empty());

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void incrementIterationCount() {
        iteration.incrementAndGet();
    }

    public void addDriveMotionMeasurements(
            double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredRobotRelativeChassisSpeeds,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    /**
     * Predicts robot's future pose based on current velocity.
     *
     * @param lookaheadTimeS How far ahead to predict (seconds)
     */
    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        Pose2d pose =
                Optional.ofNullable(getLatestFieldToRobot())
                        .map(Map.Entry::getValue)
                        .orElse(MathHelpers.kPose2dZero);
        var delta = getLatestRobotRelativeChassisSpeed().times(lookaheadTimeS);
        return pose.exp(
                new Twist2d(
                        delta.vxMetersPerSecond,
                        delta.vyMetersPerSecond,
                        delta.omegaRadiansPerSecond));
    }

    /**
     * Like getPredictedFieldToRobot but caps negative X velocity to zero. Used for non-holonomic
     * planning.
     */
    public Pose2d getPredictedCappedFieldToRobot(double lookaheadTimeS) {
        Pose2d pose =
                Optional.ofNullable(getLatestFieldToRobot())
                        .map(Map.Entry::getValue)
                        .orElse(MathHelpers.kPose2dZero);
        var delta = getLatestRobotRelativeChassisSpeed().times(lookaheadTimeS);
        return pose.exp(
                new Twist2d(
                        Math.max(0.0, delta.vxMetersPerSecond),
                        Math.max(0.0, delta.vyMetersPerSecond),
                        delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond =
                getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    private Optional<Double> getMaxAbsValueInRange(
            ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty()) return Optional.empty();
        return (Math.abs(max.get()) >= Math.abs(min.get())) ? max : min;
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(
            double minTime, double maxTime) {
        // Gyro yaw rate isn't set in sim for all models; fall back to measured omega when needed.
        if (RobotBase.isReal()) {
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        }
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(
            double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(
            double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
        lastUsedMegatagTimestamp = megatagEstimate.getTimestampSeconds();
        lastUsedMegatagPose = megatagEstimate.getVisionRobotPoseMeters();
        visionEstimateConsumer.accept(megatagEstimate);
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public Pose2d lastUsedMegatagPose() {
        return lastUsedMegatagPose;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void updateLogger() {
        if (driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/YawAngularVelocity",
                    driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/RollAngularVelocity",
                    driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/PitchAngularVelocity",
                    drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (drivePitchRads.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/PitchRads",
                    drivePitchRads.getInternalBuffer().lastEntry().getValue());
        }
        if (driveRollRads.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/RollRads",
                    driveRollRads.getInternalBuffer().lastEntry().getValue());
        }
        if (accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/AccelX", accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput(
                    "RobotState/AccelY", accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput(
                "RobotState/DesiredChassisSpeedFieldFrame",
                getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput(
                "RobotState/DesiredChassisSpeedRobotFrame",
                getLatestDesiredRobotRelativeChassisSpeeds());
        Logger.recordOutput(
                "RobotState/MeasuredChassisSpeedFieldFrame",
                getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput(
                "RobotState/FusedChassisSpeedFieldFrame",
                getLatestFusedFieldRelativeChassisSpeed());
    }

    public void setExclusiveTag(int id) {
        exclusiveTag.set(Optional.of(id));
    }

    public void clearExclusiveTag() {
        exclusiveTag.set(Optional.empty());
    }

    public Optional<Integer> getExclusiveTag() {
        return exclusiveTag.get();
    }

    public void setTrajectoryTargetPose(Pose2d pose) {
        trajectoryTargetPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryTargetPose() {
        return trajectoryTargetPose;
    }

    public void setTrajectoryCurrentPose(Pose2d pose) {
        trajectoryCurrentPose = Optional.of(pose);
    }

    public Optional<Pose2d> getTrajectoryCurrentPose() {
        return trajectoryCurrentPose;
    }

    public double getDrivePitchRadians() {
        if (drivePitchRads.getInternalBuffer().lastEntry() != null) {
            return drivePitchRads.getInternalBuffer().lastEntry().getValue();
        }
        return 0.0;
    }

    public double getDriveRollRadians() {
        if (driveRollRads.getInternalBuffer().lastEntry() != null) {
            return driveRollRads.getInternalBuffer().lastEntry().getValue();
        }
        return 0.0;
    }

    public static boolean onOpponentSide(boolean isRedAlliance, Pose2d pose) {
        return (isRedAlliance
                        && pose.getTranslation().getX()
                                < FieldConstants.fieldLength / 2 - FieldConstants.kMidlineBuffer)
                || (!isRedAlliance
                        && pose.getTranslation().getX()
                                > FieldConstants.fieldLength / 2 + FieldConstants.kMidlineBuffer);
    }

    public boolean onOpponentSide() {
        return onOpponentSide(isRedAlliance(), getLatestFieldToRobot().getValue());
    }
}
