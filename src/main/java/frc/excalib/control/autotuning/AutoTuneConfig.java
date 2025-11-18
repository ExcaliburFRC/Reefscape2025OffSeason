package frc.excalib.control.autotuning;

/**
 * Configuration for auto-tuning operations.
 * Contains parameters for the tuning process.
 * 
 * @author Excalib Team
 */
public class AutoTuneConfig {
    public final double testVoltage;
    public final double testDuration;
    public final double convergenceThreshold;
    public final AutoTuneMethod method;
    
    private AutoTuneConfig(Builder builder) {
        this.testVoltage = builder.testVoltage;
        this.testDuration = builder.testDuration;
        this.convergenceThreshold = builder.convergenceThreshold;
        this.method = builder.method;
    }
    
    /**
     * Create a default configuration for velocity control.
     * @return a velocity control configuration
     */
    public static AutoTuneConfig forVelocityControl() {
        return builder()
            .withTestVoltage(6.0)
            .withTestDuration(3.0)
            .withMethod(AutoTuneMethod.ZIEGLER_NICHOLS)
            .build();
    }
    
    /**
     * Create a default configuration for position control.
     * @return a position control configuration
     */
    public static AutoTuneConfig forPositionControl() {
        return builder()
            .withTestVoltage(4.0)
            .withTestDuration(2.0)
            .withMethod(AutoTuneMethod.NO_OVERSHOOT)
            .build();
    }
    
    /**
     * Create a new builder.
     * @return a new builder instance
     */
    public static Builder builder() {
        return new Builder();
    }
    
    public static class Builder {
        private double testVoltage = 6.0;
        private double testDuration = 3.0;
        private double convergenceThreshold = 0.01;
        private AutoTuneMethod method = AutoTuneMethod.ZIEGLER_NICHOLS;
        
        public Builder withTestVoltage(double voltage) {
            this.testVoltage = voltage;
            return this;
        }
        
        public Builder withTestDuration(double seconds) {
            this.testDuration = seconds;
            return this;
        }
        
        public Builder withConvergenceThreshold(double threshold) {
            this.convergenceThreshold = threshold;
            return this;
        }
        
        public Builder withMethod(AutoTuneMethod method) {
            this.method = method;
            return this;
        }
        
        public AutoTuneConfig build() {
            return new AutoTuneConfig(this);
        }
    }
}
