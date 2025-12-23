package frc.excalib.safety;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

/**
 * Centralized safety management system.
 * Registers and monitors multiple safety guards to prevent robot damage.
 * 
 * Example:
 * <pre>
 * SafetyManager.register(
 *     BrownoutGuard.builder().withCriticalVoltage(10.5).build(),
 *     CurrentLimitGuard.forMotor(intakeMotor, 40.0)
 * );
 * SafetyManager.getInstance().checkAll(); // Call this periodically
 * </pre>
 * 
 * @author Excalib Team
 */
public class SafetyManager {
    private static SafetyManager instance;
    
    private final List<SafetyGuard> guards = new ArrayList<>();
    private final List<String> triggeredGuards = new ArrayList<>();
    private boolean enabled = true;
    
    private SafetyManager() {}
    
    /**
     * Get the singleton instance.
     * @return the SafetyManager instance
     */
    public static SafetyManager getInstance() {
        if (instance == null) {
            instance = new SafetyManager();
        }
        return instance;
    }
    
    /**
     * Register safety guards.
     * @param guards guards to register
     */
    public static void register(SafetyGuard... guards) {
        for (SafetyGuard guard : guards) {
            getInstance().guards.add(guard);
        }
    }
    
    /**
     * Check all registered safety guards.
     * Should be called periodically (e.g., in robotPeriodic).
     * @return true if all guards pass, false if any failed
     */
    public boolean checkAll() {
        if (!enabled) return true;
        
        boolean allSafe = true;
        
        for (SafetyGuard guard : guards) {
            try {
                if (!guard.check()) {
                    allSafe = false;
                    if (!triggeredGuards.contains(guard.getName())) {
                        triggeredGuards.add(guard.getName());
                        DriverStation.reportError(
                            "Safety guard triggered: " + guard.getName() + " - " + guard.getDescription(),
                            false
                        );
                    }
                }
            } catch (Exception e) {
                DriverStation.reportError(
                    "Error checking safety guard " + guard.getName() + ": " + e.getMessage(),
                    e.getStackTrace()
                );
            }
        }
        
        return allSafe;
    }
    
    /**
     * Reset all triggered guard states.
     */
    public void reset() {
        triggeredGuards.clear();
    }
    
    /**
     * Get a list of all triggered guard names.
     * @return list of triggered guard names
     */
    public List<String> getTriggeredGuards() {
        return new ArrayList<>(triggeredGuards);
    }
    
    /**
     * Remove all registered guards.
     */
    public void clearAll() {
        guards.clear();
        triggeredGuards.clear();
    }
    
    /**
     * Enable or disable safety checks.
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    
    /**
     * Check if safety checks are enabled.
     * @return true if enabled
     */
    public boolean isEnabled() {
        return enabled;
    }
    
    /**
     * Get the number of registered guards.
     * @return guard count
     */
    public int getGuardCount() {
        return guards.size();
    }
}
