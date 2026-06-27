package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

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
        return SmartDashboard.getNumber(key, defaultValue);
    }

    public boolean hasChanged(int id) {
        double currentValue = get();
        Map<String, Double> values = lastValues.computeIfAbsent(id, unused -> new HashMap<>());
        Double previousValue = values.put(key, currentValue);
        return previousValue == null || previousValue != currentValue;
    }
}
