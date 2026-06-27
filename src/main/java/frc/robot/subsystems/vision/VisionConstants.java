package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double kLargeVariance = 1e6;

    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;

    public static final int kExpectedStdDevArrayLength = 12;

    // Camera A
    public static final double kCameraAPitchDegrees = Units.radiansToDegrees(0.4);
    public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
    public static final double kCameraAHeightOffGroundMeters = 0.2;
    public static final String kLimelightATableName = "limelight-dihlite";
    public static final double kRobotToCameraAForward = 0.2;
    public static final double kRobotToCameraASide = 0.0;
    public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(0.0);
    public static final Transform2d kRobotToCameraA =
            new Transform2d(
                    new Translation2d(kRobotToCameraAForward, kRobotToCameraASide),
                    kCameraAYawOffset);

    // Camera B
    public static final double kCameraBPitchDegrees = Units.radiansToDegrees(0.4);
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBHeightOffGroundMeters = 0.2;
    public static final String kLimelightBTableName = "limelight-dihcam";
    public static final double kRobotToCameraBForward = -0.2;
    public static final double kRobotToCameraBSide = 0.0;
    public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(180.0);
    public static final Transform2d kRobotToCameraB =
            new Transform2d(
                    new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide),
                    kCameraBYawOffset);

    // Vision processing thresholds
    public static final double kDefaultAmbiguityThreshold = 0.3;
    public static final double kDefaultYawDiffThreshold = 5.0;
    public static final double kTagAreaThresholdForYawCheck = 2.0;
    public static final double kTagMinAreaForSingleTagMegatag = 1.0;
    public static final double kDefaultZThreshold = 0.75;
    public static final double kDefaultNormThreshold = 1.0;
}
