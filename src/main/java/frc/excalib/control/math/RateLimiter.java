package frc.excalib.control.math;

/**
 * Limits the rate of change of a value.
 * Useful for smooth acceleration and preventing sudden jerks.
 * 
 * Example:
 * <pre>
 * RateLimiter accelLimiter = new RateLimiter(3.0); // 3 units per second max
 * 
 * double smoothSpeed = accelLimiter.calculate(desiredSpeed, 0.02); // 20ms loop
 * </pre>
 * 
 * @author Excalib Team
 */
public class RateLimiter {
    private final double rateLimit;
    private double previousValue = 0.0;
    private boolean initialized = false;
    
    /**
     * Creates a new RateLimiter.
     * @param rateLimit maximum rate of change per second
     */
    public RateLimiter(double rateLimit) {
        if (rateLimit <= 0) {
            throw new IllegalArgumentException("Rate limit must be positive");
        }
        this.rateLimit = rateLimit;
    }
    
    /**
     * Calculate the rate-limited value.
     * @param input desired value
     * @param dt time step in seconds
     * @return rate-limited value
     */
    public double calculate(double input, double dt) {
        if (!initialized) {
            initialized = true;
            previousValue = input;
            return input;
        }
        
        double maxChange = rateLimit * dt;
        double change = input - previousValue;
        
        if (Math.abs(change) <= maxChange) {
            previousValue = input;
            return input;
        }
        
        double limitedChange = Math.copySign(maxChange, change);
        previousValue += limitedChange;
        return previousValue;
    }
    
    /**
     * Reset the rate limiter.
     * @param value initial value
     */
    public void reset(double value) {
        previousValue = value;
        initialized = true;
    }
    
    /**
     * Reset the rate limiter to uninitialized state.
     */
    public void reset() {
        initialized = false;
        previousValue = 0.0;
    }
    
    /**
     * Get the configured rate limit.
     * @return rate limit per second
     */
    public double getRateLimit() {
        return rateLimit;
    }
}
