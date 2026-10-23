package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {

    /** Container for all vision input data. */
    class VisionIOInputs {
        /** Input data from a single camera. */
        public static class CameraInputs {
            public boolean seesTarget;
            public FiducialObservation[] fiducialObservations;
            public MegatagPoseEstimate megatagPoseEstimate;
            public MegatagPoseEstimate megatag2PoseEstimate;
            public int megatag2Count;
            public int megatagCount;
            public Pose3d pose3d;
            // Limelight 2026 stddevs: [MT1 x,y,z,roll,pitch,yaw, MT2 x,y,z,roll,pitch,yaw].
            // Empty until the camera publishes a complete, usable estimate.
            public double[] standardDeviations = new double[0];
        }

        public CameraInputs camera = new CameraInputs();
    }

    void updateInputs(VisionIOInputs inputs);
}
