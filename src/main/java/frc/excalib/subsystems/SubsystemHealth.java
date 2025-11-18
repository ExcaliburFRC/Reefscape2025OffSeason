package frc.excalib.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.excalib.telemetry.TelemetryManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Health monitoring for subsystems with automatic diagnostics and alerts.
 * Helps catch problems before they cause match failures.
 * 
 * Example:
 * <pre>
 * SubsystemHealth health = SubsystemHealth.monitor("shooter")
 *     .checkMotor("shooter_motor", shooterMotor::isConnected)
 *     .checkSensor("encoder", encoder::isConnected)
 *     .checkValue("velocity", shooterMotor::getVelocity, 0, 6000)
 *     .checkValue("temperature", shooterMotor::getTemperature, 0, 80)
 *     .build();
 * 
 * // In periodic
 * health.update();
 * if (!health.isHealthy()) {
 *     // Handle unhealthy state
 * }
 * </pre>
 * 
 * @author Excalib Team
 */
public class SubsystemHealth {
    private final String subsystemName;
    private final Map<String, HealthCheck> checks = new HashMap<>();
    private final List<String> failedChecks = new ArrayList<>();
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    private boolean isHealthy = true;
    
    private SubsystemHealth(Builder builder) {
        this.subsystemName = builder.subsystemName;
        this.checks.putAll(builder.checks);
    }
    
    /**
     * Create a health monitor for a subsystem.
     * @param subsystemName subsystem name
     * @return builder
     */
    public static Builder monitor(String subsystemName) {
        return new Builder(subsystemName);
    }
    
    /**
     * Update all health checks.
     */
    public void update() {
        failedChecks.clear();
        isHealthy = true;
        
        for (Map.Entry<String, HealthCheck> entry : checks.entrySet()) {
            String checkName = entry.getKey();
            HealthCheck check = entry.getValue();
            
            boolean passed = check.test();
            
            if (!passed) {
                isHealthy = false;
                failedChecks.add(checkName);
                
                String errorMsg = String.format(
                    "[HEALTH] %s/%s failed: %s",
                    subsystemName, checkName, check.getDescription()
                );
                
                DriverStation.reportWarning(errorMsg, false);
                telemetry.recordEvent(subsystemName + "/health", checkName + "_failed");
            }
            
            telemetry.recordBoolean(
                subsystemName + "/health/" + checkName,
                passed
            );
        }
        
        telemetry.recordBoolean(subsystemName + "/healthy", isHealthy);
    }
    
    /**
     * Check if subsystem is healthy.
     * @return true if all checks pass
     */
    public boolean isHealthy() {
        return isHealthy;
    }
    
    /**
     * Get list of failed checks.
     * @return failed check names
     */
    public List<String> getFailedChecks() {
        return new ArrayList<>(failedChecks);
    }
    
    /**
     * Builder for subsystem health monitoring.
     */
    public static class Builder {
        private final String subsystemName;
        private final Map<String, HealthCheck> checks = new HashMap<>();
        
        private Builder(String subsystemName) {
            this.subsystemName = subsystemName;
        }
        
        /**
         * Check if a motor is connected.
         * @param motorName motor name
         * @param isConnected connection check
         * @return this builder
         */
        public Builder checkMotor(String motorName, BooleanSupplier isConnected) {
            checks.put(motorName + "_connected", new HealthCheck(
                isConnected,
                motorName + " is not connected"
            ));
            return this;
        }
        
        /**
         * Check if a sensor is connected.
         * @param sensorName sensor name
         * @param isConnected connection check
         * @return this builder
         */
        public Builder checkSensor(String sensorName, BooleanSupplier isConnected) {
            checks.put(sensorName + "_connected", new HealthCheck(
                isConnected,
                sensorName + " is not connected"
            ));
            return this;
        }
        
        /**
         * Check if a value is within range.
         * @param valueName value name
         * @param valueSupplier value supplier
         * @param minValue minimum allowed value
         * @param maxValue maximum allowed value
         * @return this builder
         */
        public Builder checkValue(String valueName, 
                                 java.util.function.DoubleSupplier valueSupplier,
                                 double minValue, 
                                 double maxValue) {
            checks.put(valueName + "_range", new HealthCheck(
                () -> {
                    double value = valueSupplier.getAsDouble();
                    return value >= minValue && value <= maxValue;
                },
                String.format("%s out of range (%.2f - %.2f)", valueName, minValue, maxValue)
            ));
            return this;
        }
        
        /**
         * Add a custom health check.
         * @param checkName check name
         * @param condition check condition
         * @param description description of what this checks
         * @return this builder
         */
        public Builder check(String checkName, BooleanSupplier condition, String description) {
            checks.put(checkName, new HealthCheck(condition, description));
            return this;
        }
        
        /**
         * Build the health monitor.
         * @return configured health monitor
         */
        public SubsystemHealth build() {
            return new SubsystemHealth(this);
        }
    }
    
    private static class HealthCheck {
        private final BooleanSupplier condition;
        private final String description;
        
        HealthCheck(BooleanSupplier condition, String description) {
            this.condition = condition;
            this.description = description;
        }
        
        boolean test() {
            try {
                return condition.getAsBoolean();
            } catch (Exception e) {
                return false;
            }
        }
        
        String getDescription() {
            return description;
        }
    }
}
