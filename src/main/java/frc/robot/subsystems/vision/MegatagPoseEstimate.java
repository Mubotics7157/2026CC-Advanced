package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.lib.limelight.LimelightHelpers;
import frc.robot.lib.util.MathHelpers;
import java.nio.ByteBuffer;

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
        int[] fiducialIds)
        implements StructSerializable {

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

    public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();

    public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "record:MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + 3 * Double.BYTES;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot; double timestampSeconds; double latency; double avgTagArea";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
        }

        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            Pose2d fieldToRobot = Pose2d.struct.unpack(bb);
            double timestampSeconds = bb.getDouble();
            double latency = bb.getDouble();
            double avgTagArea = bb.getDouble();
            double quality = bb.getDouble();
            int[] fiducialIds = new int[0];
            return new MegatagPoseEstimate(
                    fieldToRobot, timestampSeconds, latency, avgTagArea, quality, fiducialIds);
        }

        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot());
            bb.putDouble(value.timestampSeconds());
            bb.putDouble(value.latency());
            bb.putDouble(value.avgTagArea());
            bb.putDouble(value.quality());
        }

        @Override
        public String getTypeName() {
            return "MegatagPoseEstimate";
        }
    }
}
