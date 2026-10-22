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

    // This robot has one Limelight. Keep its measured mounting values here so real and
    // simulated vision share the identical robot-to-camera transform.
    public static final double kCameraPitchDegrees = Units.radiansToDegrees(0.4);
    public static final double kCameraPitchRads = Units.degreesToRadians(kCameraPitchDegrees);
    public static final double kCameraHeightOffGroundMeters = 0.2;
    public static final String kLimelightTableName = "limelight-dihlite";
    public static final double kRobotToCameraForward = 0.2;
    public static final double kRobotToCameraSide = 0.0;
    public static final Rotation2d kCameraYawOffset = Rotation2d.fromDegrees(0.0);
    public static final Transform2d kRobotToCamera =
            new Transform2d(
                    new Translation2d(kRobotToCameraForward, kRobotToCameraSide), kCameraYawOffset);

    // Vision processing thresholds
    public static final double kDefaultAmbiguityThreshold = 0.3;
    public static final double kDefaultYawDiffThreshold = 5.0;
    public static final double kTagAreaThresholdForYawCheck = 2.0;
    public static final double kTagMinAreaForSingleTagMegatag = 1.0;
    public static final double kDefaultZThreshold = 0.75;
    public static final double kDefaultNormThreshold = 1.0;
}
