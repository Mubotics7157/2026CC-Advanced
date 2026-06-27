package frc.robot.lib.subsystems;

public interface CanCoderIO {
    void readInputs(CanCoderInputs inputs);

    void updateFrequency(double hz);
}
