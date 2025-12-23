package frc.excalib.control.math;

import java.util.TreeMap;

/**
 * Interpolates between data points for lookups like shooter RPM tables.
 * Uses linear interpolation between the two nearest points.
 * 
 * Common use case: distance-to-RPM lookup tables for shooters
 * 
 * Example:
 * <pre>
 * Interpolator shooterTable = new Interpolator();
 * shooterTable.put(1.0, 2000);  // At 1m, shoot at 2000 RPM
 * shooterTable.put(2.0, 2500);  // At 2m, shoot at 2500 RPM
 * shooterTable.put(3.0, 3000);  // At 3m, shoot at 3000 RPM
 * 
 * double rpm = shooterTable.get(1.5); // Returns 2250 RPM (interpolated)
 * </pre>
 * 
 * @author Excalib Team
 */
public class Interpolator {
    private final TreeMap<Double, Double> data = new TreeMap<>();
    
    /**
     * Add a data point to the interpolation table.
     * @param key the independent variable (e.g., distance)
     * @param value the dependent variable (e.g., RPM)
     */
    public void put(double key, double value) {
        data.put(key, value);
    }
    
    /**
     * Get an interpolated value.
     * If the key is outside the data range, returns the nearest value.
     * 
     * @param key the independent variable
     * @return the interpolated dependent variable
     */
    public double get(double key) {
        if (data.isEmpty()) {
            throw new IllegalStateException("No data points in interpolator");
        }
        
        // Check if exact match
        if (data.containsKey(key)) {
            return data.get(key);
        }
        
        // Get surrounding points
        Double lowerKey = data.floorKey(key);
        Double upperKey = data.ceilingKey(key);
        
        // Handle edge cases
        if (lowerKey == null) {
            return data.get(upperKey);
        }
        if (upperKey == null) {
            return data.get(lowerKey);
        }
        
        // Linear interpolation
        double lowerValue = data.get(lowerKey);
        double upperValue = data.get(upperKey);
        
        double ratio = (key - lowerKey) / (upperKey - lowerKey);
        return lowerValue + ratio * (upperValue - lowerValue);
    }
    
    /**
     * Get the number of data points.
     * @return data point count
     */
    public int size() {
        return data.size();
    }
    
    /**
     * Clear all data points.
     */
    public void clear() {
        data.clear();
    }
    
    /**
     * Check if the interpolator has data points.
     * @return true if empty
     */
    public boolean isEmpty() {
        return data.isEmpty();
    }
}
