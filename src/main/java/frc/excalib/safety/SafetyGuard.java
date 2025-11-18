package frc.excalib.safety;

/**
 * Base interface for safety guards that monitor system health and prevent damage.
 * 
 * @author Excalib Team
 */
public interface SafetyGuard {
    /**
     * Check if the guard condition is violated.
     * @return true if safe, false if guard triggered
     */
    boolean check();
    
    /**
     * Get a description of what this guard protects against.
     * @return description
     */
    String getDescription();
    
    /**
     * Called when the guard is triggered.
     * Should take corrective action to prevent damage.
     */
    void onTriggered();
    
    /**
     * Get the guard's name for logging.
     * @return guard name
     */
    String getName();
}
