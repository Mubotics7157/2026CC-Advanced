package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class DriveConstants {
    public static final Transform2d kDriveToCoralOffset =
            new Transform2d(new Translation2d(0.66, 0.0), Rotation2d.kZero);

    public static final double kDriveMaxSpeed = 6.14;
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxXAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxYAccelerationMetersPerSecondSquared = 10.0;
    public static final double kDriveMaxAngularRate = 16.475;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;

    public static final double kHeadingControllerP = 5.0;
    public static final double kHeadingControllerI = 0.0;
    public static final double kHeadingControllerD = 0.0;

    public static final CommandSwerveDrivetrain kDrivetrain =
            RobotBase.isSimulation()
                    ? CompTunerConstants.createDrivetrain()
                    : (Constants.kIsPracticeBot
                            ? PracTunerConstants.createDrivetrain()
                            : CompTunerConstants.createDrivetrain());

    // Used by MapleSim configuration
    public static final double kRobotWeightPounds = 86.0;
    public static final double kBumperLengthInches = 32.0;
    public static final double kBumperWidthInches = 32.0;
    public static final double kWheelCoefficientOfFriction = 1.48;
    public static final int kDriveMotorCount = 1;

    // Odometry/vision standard deviations
    public static final double kDisabledDriveXStdDev = 1.0;
    public static final double kDisabledDriveYStdDev = 1.0;
    public static final double kDisabledDriveRotStdDev = 1.0;

    public static final double kEnabledDriveXStdDev = 0.3;
    public static final double kEnabledDriveYStdDev = 0.3;
    public static final double kEnabledDriveRotStdDev = 0.2;
}
