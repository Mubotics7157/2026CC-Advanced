package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** A SmartDashboard number that records every effective read to the active AdvantageKit log. */
public class LoggedTunableNumber {
    private static final Map<Integer, Map<String, Double>> lastValues = new HashMap<>();

    private final String key;
    private final double defaultValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        SmartDashboard.putNumber(key, defaultValue);
    }

    public double get() {
        double value = SmartDashboard.getNumber(key, defaultValue);
        if (!Double.isFinite(value)) {
            value = defaultValue;
        }
        Logger.recordOutput(key, value);
        return value;
    }

    /** Returns the dashboard value clamped to a safe range and logs the effective value. */
    public double getClamped(double min, double max) {
        if (!Double.isFinite(min) || !Double.isFinite(max) || max < min) {
            return get();
        }
        double value = Math.max(min, Math.min(max, get()));
        Logger.recordOutput(key + "/Effective", value);
        return value;
    }

    public boolean hasChanged(int id) {
        double currentValue = get();
        Map<String, Double> values = lastValues.computeIfAbsent(id, unused -> new HashMap<>());
        Double previousValue = values.put(key, currentValue);
        return previousValue == null || previousValue != currentValue;
    }
}
