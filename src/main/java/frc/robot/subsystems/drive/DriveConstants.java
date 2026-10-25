package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class DriveConstants {
    private static final boolean kUsePracticeConfiguration =
            !RobotBase.isSimulation() && Constants.kIsPracticeBot;

    /** Selected Tuner X configuration values; these do not construct a drivetrain instance. */
    public static final SwerveDrivetrainConstants kDrivetrainConstants =
            kUsePracticeConfiguration
                    ? PracTunerConstants.DrivetrainConstants
                    : CompTunerConstants.DrivetrainConstants;

    private static final SwerveModuleConstants<?, ?, ?>[] kSelectedModuleConstants =
            kUsePracticeConfiguration
                    ? new SwerveModuleConstants<?, ?, ?>[] {
                        PracTunerConstants.FrontLeft,
                        PracTunerConstants.FrontRight,
                        PracTunerConstants.BackLeft,
                        PracTunerConstants.BackRight
                    }
                    : new SwerveModuleConstants<?, ?, ?>[] {
                        CompTunerConstants.FrontLeft,
                        CompTunerConstants.FrontRight,
                        CompTunerConstants.BackLeft,
                        CompTunerConstants.BackRight
                    };

    /** Returns the selected module constants without exposing the backing array. */
    public static SwerveModuleConstants<?, ?, ?>[] getModuleConstants() {
        return kSelectedModuleConstants.clone();
    }

    /** Maximum wheel speed at 12 V from the active 2026 Tuner X configuration. */
    public static final double kDriveMaxSpeed =
            (kUsePracticeConfiguration
                            ? PracTunerConstants.kSpeedAt12Volts
                            : CompTunerConstants.kSpeedAt12Volts)
                    .in(MetersPerSecond);

    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxXAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxYAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;

    /**
     * Maximum angular velocity of a swerve module's azimuth pivot, in radians per second. Used by
     * the {@code SwerveSetpointGenerator} to limit how fast modules can rotate between setpoints.
     */
    public static final double kMaxSteerVelocityRadPerSec = 12.0;

    public static final double kHeadingControllerP = 5.0;
    public static final double kHeadingControllerI = 0.0;
    public static final double kHeadingControllerD = 0.0;

    /** Furthest module radius from the robot center, derived from the active module layout. */
    public static final double kDriveBaseRadiusMeters = calculateDriveBaseRadiusMeters();

    /** Maximum chassis yaw rate when every module reaches its configured maximum wheel speed. */
    public static final double kDriveMaxAngularRate = kDriveMaxSpeed / kDriveBaseRadiusMeters;

    private static double calculateDriveBaseRadiusMeters() {
        double radius = 0.0;
        for (var module : kSelectedModuleConstants) {
            radius = Math.max(radius, Math.hypot(module.LocationX, module.LocationY));
        }
        return radius;
    }

    // Used by MapleSim configuration
    public static final double kRobotWeightPounds =
            edu.wpi.first.units.Units.Kilograms.of(Constants.kRobotMassKg)
                    .in(edu.wpi.first.units.Units.Pounds);
    public static final double kBumperLengthInches = 34.5;
    public static final double kBumperWidthInches = 32.5;
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
