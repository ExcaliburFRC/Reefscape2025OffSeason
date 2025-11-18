package frc.excalib.control.math;

import edu.wpi.first.wpilibj.Timer;

/**
 * Debounces a boolean signal to prevent flickering.
 * Useful for sensors, buttons, and vision detection that may be noisy.
 * 
 * Example:
 * <pre>
 * Debouncer gamePieceDebouncer = new Debouncer(0.1); // 100ms debounce
 * 
 * if (gamePieceDebouncer.calculate(sensor.hasGamePiece())) {
 *     // Confirmed: game piece is present
 * }
 * </pre>
 * 
 * @author Excalib Team
 */
public class Debouncer {
    private final double debounceTime;
    private boolean lastValue = false;
    private double changeTime = 0.0;
    
    /**
     * Creates a new Debouncer.
     * @param debounceTimeSeconds time the value must be stable before changing
     */
    public Debouncer(double debounceTimeSeconds) {
        if (debounceTimeSeconds < 0) {
            throw new IllegalArgumentException("Debounce time must be non-negative");
        }
        this.debounceTime = debounceTimeSeconds;
    }
    
    /**
     * Calculate the debounced value.
     * @param value current value
     * @return debounced value
     */
    public boolean calculate(boolean value) {
        double currentTime = Timer.getFPGATimestamp();
        
        if (value != lastValue) {
            // Value changed, record the time
            changeTime = currentTime;
            lastValue = value;
            return !value; // Return the old value until debounced
        }
        
        // Value is stable, check if enough time has passed
        if (currentTime - changeTime >= debounceTime) {
            return value;
        }
        
        // Still in debounce period, return old value
        return !value;
    }
    
    /**
     * Reset the debouncer state.
     */
    public void reset() {
        lastValue = false;
        changeTime = 0.0;
    }
    
    /**
     * Get the configured debounce time.
     * @return debounce time in seconds
     */
    public double getDebounceTime() {
        return debounceTime;
    }
}
