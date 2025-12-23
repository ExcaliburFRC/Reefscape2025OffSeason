package frc.excalib.telemetry;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

/**
 * Centralized telemetry management for logging, metrics, and diagnostics.
 * Provides easy-to-use APIs for tracking performance and debugging issues.
 * 
 * Example:
 * <pre>
 * TelemetryManager.getInstance()
 *     .recordValue("drivetrain/velocity", velocity)
 *     .recordLatency("vision/processing")
 *     .recordEvent("auto/milestone", "reached_scoring_position");
 * </pre>
 * 
 * @author Excalib Team
 */
public class TelemetryManager {
    private static TelemetryManager instance;
    
    private final Map<String, Double> latencyTimers = new HashMap<>();
    private final Map<String, Long> eventCounts = new HashMap<>();
    private final Map<String, Double> maxValues = new HashMap<>();
    private final Map<String, Double> minValues = new HashMap<>();
    
    private boolean enabled = true;

    private TelemetryManager() {}

    /**
     * Get the singleton instance.
     * @return the TelemetryManager instance
     */
    public static TelemetryManager getInstance() {
        if (instance == null) {
            instance = new TelemetryManager();
        }
        return instance;
    }

    /**
     * Record a numeric value to telemetry.
     * @param key the telemetry key (e.g., "drivetrain/velocity")
     * @param value the value to record
     * @return this for method chaining
     */
    public TelemetryManager recordValue(String key, double value) {
        if (!enabled) return this;
        
        SmartDashboard.putNumber(key, value);
        
        // Track min/max
        maxValues.put(key, Math.max(maxValues.getOrDefault(key, Double.NEGATIVE_INFINITY), value));
        minValues.put(key, Math.min(minValues.getOrDefault(key, Double.POSITIVE_INFINITY), value));
        
        return this;
    }

    /**
     * Record a boolean value to telemetry.
     * @param key the telemetry key
     * @param value the value to record
     * @return this for method chaining
     */
    public TelemetryManager recordBoolean(String key, boolean value) {
        if (!enabled) return this;
        SmartDashboard.putBoolean(key, value);
        return this;
    }

    /**
     * Record a string value to telemetry.
     * @param key the telemetry key
     * @param value the value to record
     * @return this for method chaining
     */
    public TelemetryManager recordString(String key, String value) {
        if (!enabled) return this;
        SmartDashboard.putString(key, value);
        return this;
    }

    /**
     * Start timing a latency measurement.
     * @param key the operation key (e.g., "vision/processing")
     * @return this for method chaining
     */
    public TelemetryManager startLatencyTimer(String key) {
        if (!enabled) return this;
        latencyTimers.put(key, Timer.getFPGATimestamp());
        return this;
    }

    /**
     * End timing and record the latency.
     * @param key the operation key
     * @return the measured latency in seconds, or -1 if timer wasn't started
     */
    public double recordLatency(String key) {
        if (!enabled) return -1.0;
        
        Double startTime = latencyTimers.remove(key);
        if (startTime == null) {
            return -1.0;
        }
        
        double latency = Timer.getFPGATimestamp() - startTime;
        SmartDashboard.putNumber(key + "/latency_ms", latency * 1000.0);
        return latency;
    }

    /**
     * Record an event occurrence.
     * @param category the event category (e.g., "auto/milestone")
     * @param event the event name
     * @return this for method chaining
     */
    public TelemetryManager recordEvent(String category, String event) {
        if (!enabled) return this;
        
        String key = category + "/" + event;
        long count = eventCounts.getOrDefault(key, 0L) + 1;
        eventCounts.put(key, count);
        
        SmartDashboard.putString(category + "/last_event", event);
        SmartDashboard.putNumber(key + "/count", count);
        
        return this;
    }

    /**
     * Get the maximum value recorded for a key.
     * @param key the telemetry key
     * @return the maximum value, or null if not tracked
     */
    public Double getMaxValue(String key) {
        return maxValues.get(key);
    }

    /**
     * Get the minimum value recorded for a key.
     * @param key the telemetry key
     * @return the minimum value, or null if not tracked
     */
    public Double getMinValue(String key) {
        return minValues.get(key);
    }

    /**
     * Get the event count for a specific event.
     * @param category the event category
     * @param event the event name
     * @return the count, or 0 if not recorded
     */
    public long getEventCount(String category, String event) {
        return eventCounts.getOrDefault(category + "/" + event, 0L);
    }

    /**
     * Reset all telemetry data.
     */
    public void reset() {
        latencyTimers.clear();
        eventCounts.clear();
        maxValues.clear();
        minValues.clear();
    }

    /**
     * Enable or disable telemetry recording.
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Check if telemetry is enabled.
     * @return true if enabled
     */
    public boolean isEnabled() {
        return enabled;
    }
}
