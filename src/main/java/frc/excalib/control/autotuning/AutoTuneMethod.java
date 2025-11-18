package frc.excalib.control.autotuning;

/**
 * Auto-tuning methods for PID controllers.
 * Each method has different characteristics and is suitable for different applications.
 * 
 * @author Excalib Team
 */
public enum AutoTuneMethod {
    /**
     * Ziegler-Nichols method: Good general-purpose tuning.
     * Works by finding the ultimate gain and period, then applying standard ratios.
     */
    ZIEGLER_NICHOLS,
    
    /**
     * Tyreus-Luyben method: More conservative than Ziegler-Nichols.
     * Better for systems that can't tolerate overshoot.
     */
    TYREUS_LUYBEN,
    
    /**
     * Cohen-Coon method: Good for systems with significant lag.
     * Uses step response characteristics.
     */
    COHEN_COON,
    
    /**
     * No overshoot method: Prioritizes stability over response time.
     * Good for mechanisms where overshoot could cause damage.
     */
    NO_OVERSHOOT
}
