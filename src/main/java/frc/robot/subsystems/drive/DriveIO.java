package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.util.MathHelpers;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code DriveIO} interface defines the input/output operations for the drivetrain. It provides
 * methods to control and monitor the swerve drive system.
 *
 * <p>Brains live in {@link DriveSubsystem}; this layer maps to the physical CTRE drivetrain
 * (hardware) or replays a recorded log (replay). All characterization entry points are defined as
 * {@code default} no-ops so replay and any future IO implementation remain source-compatible.
 */
public interface DriveIO {

    @AutoLog
    class DriveIOInputs {
        public Pose2d pose = MathHelpers.kPose2dZero;
        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {};
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {};
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {};
        public double[] moduleDriveAppliedVolts = new double[] {};
        public double[] moduleSteerAppliedVolts = new double[] {};
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public double odometryPeriod = 0.0;

        public double gyroYawDeg = 0.0;
        public Rotation2d[] moduleAbsolutePositions = new Rotation2d[] {};
    }

    void updateInputs(DriveIOInputs inputs);

    void resetOdometry(Pose2d pose);

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);

    void setStateStdDevs(double xStd, double yStd, double rotStd);

    // -------------------------------------------------------------------------
    // Drivetrain characterization (SysId) entry points.
    //
    // These intentionally mirror the {@link com.ctre.phoenix6.swerve.SwerveRequest}
    // SysId* requests so the subsystem can drive the drivetrain through a voltage in
    // the proper frame. Replay-only or stub implementations may leave these alone.
    // -------------------------------------------------------------------------

    /**
     * Apply an open-loop voltage to the drive motors with the modules steered forward (0 rad).
     *
     * <p>This is the same behavior as {@link
     * com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation} and is used for the
     * translation (drive) SysId test.
     *
     * @param volts Voltage to apply to every drive motor.
     */
    default void setDriveOpenLoopVoltage(Voltage volts) {}

    /**
     * Command every drive motor to a closed-loop (velocity) setpoint given a robot-relative chassis
     * speed in wheel linear velocity terms. Used by wheel-radius characterization.
     *
     * @param speeds Robot-relative chassis speeds.
     */
    default void setDriveVelocity(ChassisSpeeds speeds) {}

    /**
     * Apply an open-loop voltage to the steer (azimuth) motors. Used to characterize the steer
     * modules (kS / kV / kA of the azimuth pivot).
     *
     * @param volts Voltage to apply to every steer motor.
     */
    default void setSteerOpenLoopVoltage(Voltage volts) {}

    /**
     * Command a robot-relative angular velocity with steady-state wheel speeds. Used for the
     * rotation (heading controller / MOI) SysId test.
     *
     * @param rotationRate Robot-relative angular velocity.
     */
    default void setRobotRotationRate(AngularVelocity rotationRate) {}

    /**
     * @return The cumulative drive-axle (wheel) angle of each module in radians, measured from the
     *     drive motor's raw position sensor and the configured gear ratio.
     *     <p>This intentionally does <b>not</b> use the odometry distance (which is already scaled
     *     by the configured wheel radius), so the wheel-radius characterization can solve for the
     *     true radius independently.
     */
    default double[] getWheelRadiusCharacterizationPositions() {
        return new double[] {0.0, 0.0, 0.0, 0.0};
    }

    /**
     * @return The azimuth (steer) angle of each module, in radians. Used by the steer SysId test to
     *     record position.
     */
    default double[] getModuleSteerPositionsRad() {
        return new double[] {0.0, 0.0, 0.0, 0.0};
    }

    /**
     * @return The azimuth (steer) angular velocity of each module, in radians/sec. Used by the
     *     steer SysId test to record velocity.
     */
    default double[] getModuleSteerVelocitiesRadPerSec() {
        return new double[] {0.0, 0.0, 0.0, 0.0};
    }

    /**
     * @return The average drive-axle angular velocity of all four modules, in radians/sec.
     */
    default double getFFCharacterizationVelocity() {
        return 0.0;
    }

    /**
     * @return The current gyro yaw as a Rotation2d.
     */
    default Rotation2d getRotation() {
        return new Rotation2d();
    }
}
