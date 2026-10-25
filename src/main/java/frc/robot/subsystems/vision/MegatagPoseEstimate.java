package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.limelight.LimelightHelpers;
import frc.robot.lib.util.MathHelpers;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record MegatagPoseEstimate(
        Pose2d fieldToRobot,
        double timestampSeconds,
        double latency,
        double avgTagArea,
        double quality,
        int[] fiducialIds) {

    public MegatagPoseEstimate {
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.kPose2dZero;
        }
        if (fiducialIds == null) {
            fiducialIds = new int[0];
        }
    }

    /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        Pose2d fieldToRobot = poseEstimate.pose;
        if (fieldToRobot == null) {
            fieldToRobot = MathHelpers.kPose2dZero;
        }
        var rawFiducials =
                poseEstimate.rawFiducials == null
                        ? new LimelightHelpers.RawFiducial[0]
                        : poseEstimate.rawFiducials;
        int[] fiducialIds = new int[rawFiducials.length];
        for (int i = 0; i < rawFiducials.length; i++) {
            if (rawFiducials[i] != null) {
                fiducialIds[i] = rawFiducials[i].id;
            }
        }
        double quality =
                fiducialIds.length > 1
                        ? 1.0
                        : fiducialIds.length == 1 && rawFiducials[0] != null
                                ? 1.0 - rawFiducials[0].ambiguity
                                : 0.0;
        return new MegatagPoseEstimate(
                fieldToRobot,
                poseEstimate.timestampSeconds,
                poseEstimate.latency,
                poseEstimate.avgTagArea,
                quality,
                fiducialIds);
    }
}
