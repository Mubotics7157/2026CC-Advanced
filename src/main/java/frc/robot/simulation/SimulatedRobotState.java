package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.RobotState;
import frc.robot.lib.time.RobotTime;

/**
 * Minimal simulation-side robot state.
 *
 * <p>Provides the simulated ground-truth robot pose for drive + vision simulation.
 */
public class SimulatedRobotState {
    private final TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth =
            TimeInterpolatableBuffer.createBuffer(RobotState.LOOKBACK_TIME);

    public synchronized void addFieldToRobot(Pose2d pose) {
        fieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
        if (entry == null) {
            return null;
        }
        return entry.getValue();
    }
}
