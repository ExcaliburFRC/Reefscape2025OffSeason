package frc.excalib.safety;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Monitors battery voltage and reduces non-critical loads during brownout conditions.
 * Prevents robot disconnections during high-current events.
 * 
 * Example:
 * <pre>
 * SafetyManager.register(BrownoutGuard.builder()
 *     .withWarningVoltage(11.5)
 *     .withCriticalVoltage(10.5)
 *     .build());
 * </pre>
 * 
 * @author Excalib Team
 */
public class BrownoutGuard implements SafetyGuard {
    private final double warningVoltage;
    private final double criticalVoltage;
    private boolean inWarning = false;
    private boolean inCritical = false;
    private Runnable onWarning;
    private Runnable onCritical;
    
    private BrownoutGuard(Builder builder) {
        this.warningVoltage = builder.warningVoltage;
        this.criticalVoltage = builder.criticalVoltage;
        this.onWarning = builder.onWarning;
        this.onCritical = builder.onCritical;
    }
    
    @Override
    public boolean check() {
        double voltage = RobotController.getBatteryVoltage();
        
        if (voltage < criticalVoltage && !inCritical) {
            inCritical = true;
            inWarning = true;
            onTriggered();
            if (onCritical != null) {
                onCritical.run();
            }
            return false;
        } else if (voltage < warningVoltage && !inWarning) {
            inWarning = true;
            if (onWarning != null) {
                onWarning.run();
            }
        } else if (voltage >= warningVoltage) {
            inWarning = false;
            inCritical = false;
        }
        
        return voltage >= criticalVoltage;
    }
    
    @Override
    public String getDescription() {
        return "Monitors battery voltage and prevents brownouts";
    }
    
    @Override
    public void onTriggered() {
        System.err.println("[BROWNOUT] Critical battery voltage detected: " + 
                          RobotController.getBatteryVoltage() + "V");
    }
    
    @Override
    public String getName() {
        return "BrownoutGuard";
    }
    
    /**
     * Check if in warning state (voltage below warning threshold).
     * @return true if in warning
     */
    public boolean isInWarning() {
        return inWarning;
    }
    
    /**
     * Check if in critical state (voltage below critical threshold).
     * @return true if in critical state
     */
    public boolean isInCritical() {
        return inCritical;
    }
    
    public static Builder builder() {
        return new Builder();
    }
    
    public static class Builder {
        private double warningVoltage = 11.5;
        private double criticalVoltage = 10.5;
        private Runnable onWarning = null;
        private Runnable onCritical = null;
        
        public Builder withWarningVoltage(double voltage) {
            this.warningVoltage = voltage;
            return this;
        }
        
        public Builder withCriticalVoltage(double voltage) {
            this.criticalVoltage = voltage;
            return this;
        }
        
        public Builder onWarning(Runnable action) {
            this.onWarning = action;
            return this;
        }
        
        public Builder onCritical(Runnable action) {
            this.onCritical = action;
            return this;
        }
        
        public BrownoutGuard build() {
            return new BrownoutGuard(this);
        }
    }
}
