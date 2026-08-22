package frc.robot.lib.util;

import java.util.Map;
import java.util.TreeMap;

/** TreeMap-backed double lookup table with linear interpolation and endpoint clamping. */
public class InterpolatingTreeMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();

    public InterpolatingTreeMap put(double key, double value) {
        map.put(key, value);
        return this;
    }

    public double get(double key) {
        if (map.isEmpty()) {
            return 0.0;
        }

        Double exact = map.get(key);
        if (exact != null) {
            return exact;
        }

        Map.Entry<Double, Double> floor = map.floorEntry(key);
        Map.Entry<Double, Double> ceiling = map.ceilingEntry(key);

        if (floor == null) {
            return ceiling.getValue();
        }
        if (ceiling == null) {
            return floor.getValue();
        }

        double t = (key - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return floor.getValue() + t * (ceiling.getValue() - floor.getValue());
    }

    public boolean isEmpty() {
        return map.isEmpty();
    }

    public int size() {
        return map.size();
    }

    public InterpolatingTreeMap clear() {
        map.clear();
        return this;
    }

    public double getMinKey() {
        return map.isEmpty() ? 0.0 : map.firstKey();
    }

    public double getMaxKey() {
        return map.isEmpty() ? 0.0 : map.lastKey();
    }

    public TreeMap<Double, Double> getMap() {
        return map;
    }
}
