package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.lib.limelight.LimelightHelpers;

/** Hardware implementation of VisionIO using the robot's single Limelight. */
public class VisionIOHardwareLimelight implements VisionIO {
    private final NetworkTable table =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight() {
        setLLSettings();
    }

    /** Configures Limelight camera poses in robot coordinate system. */
    private void setLLSettings() {
        double[] cameraPose = {
            VisionConstants.kRobotToCameraForward,
            VisionConstants.kRobotToCameraSide,
            VisionConstants.kCameraHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraPitchDegrees,
            VisionConstants.kCameraYawOffset.getDegrees()
        };
        table.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPose);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        readCameraData(table, inputs.camera, VisionConstants.kLimelightTableName);
    }

    /** Reads data from a single Limelight camera. */
    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        camera.megatagPoseEstimate = null;
        camera.megatag2PoseEstimate = null;
        camera.megatagCount = 0;
        camera.megatag2Count = 0;
        camera.pose3d = null;
        camera.fiducialObservations = new FiducialObservation[0];
        camera.standardDeviations = new double[0];
        if (camera.seesTarget) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var robotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (megatag != null) {
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatagCount = megatag.tagCount;
                    camera.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }

                camera.standardDeviations = table.getEntry("stddevs").getDoubleArray(new double[0]);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
}
