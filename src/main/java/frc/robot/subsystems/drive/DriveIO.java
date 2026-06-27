package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.util.MathHelpers;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * The {@code DriveIO} interface defines the input/output operations for the drivetrain. It provides
 * methods to control and monitor the swerve drive system.
 */
public interface DriveIO {

    @AutoLog
    class DriveIOInputs {
        public Pose2d pose = MathHelpers.kPose2dZero;
        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {};
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {};
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
}
