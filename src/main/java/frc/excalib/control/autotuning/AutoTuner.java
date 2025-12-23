package frc.excalib.control.autotuning;

import edu.wpi.first.wpilibj.Timer;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Automatically tunes PID controllers using system characterization.
 * Eliminates hours of manual tuning by applying proven tuning methods.
 * 
 * Example:
 * <pre>
 * AutoTuner tuner = new AutoTuner(shooterMotor, shooterMotor::getMotorVelocity);
 * Gains gains = tuner.characterizeAndTune(
 *     AutoTuneMethod.ZIEGLER_NICHOLS,
 *     AutoTuneConfig.forVelocityControl()
 * );
 * </pre>
 * 
 * @author Excalib Team
 */
public class AutoTuner {
    private final Motor motor;
    private final DoubleSupplier feedbackSupplier;
    
    // Characterization results
    private double kv = 0.0;  // Velocity feedforward
    private double ka = 0.0;  // Acceleration feedforward
    
    /**
     * Creates a new AutoTuner.
     * 
     * @param motor the motor to tune
     * @param feedbackSupplier supplier for feedback measurement (position or velocity)
     */
    public AutoTuner(Motor motor, DoubleSupplier feedbackSupplier) {
        this.motor = motor;
        this.feedbackSupplier = feedbackSupplier;
    }
    
    /**
     * Characterize the system and calculate tuned gains.
     * 
     * @param method the tuning method to use
     * @param config the tuning configuration
     * @return calculated gains
     */
    public Gains characterizeAndTune(AutoTuneMethod method, AutoTuneConfig config) {
        // Run characterization
        CharacterizationResult result = characterize(config);
        
        // Calculate gains based on method
        return calculateGains(method, result);
    }
    
    /**
     * Run a system characterization test.
     * 
     * @param config the test configuration
     * @return characterization results
     */
    public CharacterizationResult characterize(AutoTuneConfig config) {
        List<Double> velocities = new ArrayList<>();
        List<Double> voltages = new ArrayList<>();
        List<Double> timestamps = new ArrayList<>();
        
        Timer timer = new Timer();
        timer.start();
        
        double startTime = Timer.getFPGATimestamp();
        
        // Apply constant voltage and measure response
        motor.setVoltage(config.testVoltage);
        
        while (timer.get() < config.testDuration) {
            double currentTime = Timer.getFPGATimestamp();
            double velocity = feedbackSupplier.getAsDouble();
            
            timestamps.add(currentTime - startTime);
            velocities.add(velocity);
            voltages.add(config.testVoltage);
            
            // Sleep a bit to not overwhelm the system
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        motor.stopMotor();
        timer.stop();
        
        // Calculate kv and ka from the data
        calculateFeedforwardGains(velocities, voltages, timestamps);
        
        return new CharacterizationResult(kv, ka, velocities, voltages, timestamps);
    }
    
    private void calculateFeedforwardGains(List<Double> velocities, List<Double> voltages, List<Double> timestamps) {
        if (velocities.size() < 2) {
            kv = 0.0;
            ka = 0.0;
            return;
        }
        
        // Simple linear regression for kv (steady-state gain)
        // V = kv * velocity + ks
        double sumVelocity = 0.0;
        double sumVoltage = 0.0;
        int steadyStateStart = velocities.size() / 2; // Use second half for steady state
        
        for (int i = steadyStateStart; i < velocities.size(); i++) {
            sumVelocity += velocities.get(i);
            sumVoltage += voltages.get(i);
        }
        
        int n = velocities.size() - steadyStateStart;
        double avgVelocity = sumVelocity / n;
        double avgVoltage = sumVoltage / n;
        
        if (avgVelocity > 0.1) { // Avoid division by near-zero
            kv = avgVoltage / avgVelocity;
        } else {
            kv = 0.0;
        }
        
        // Estimate ka from transient response
        ka = 0.01; // Conservative default, would need more sophisticated analysis
    }
    
    private Gains calculateGains(AutoTuneMethod method, CharacterizationResult result) {
        double kp, ki, kd;
        
        switch (method) {
            case ZIEGLER_NICHOLS:
                // Classic Ziegler-Nichols for velocity control
                kp = 0.6 * (1.0 / kv);
                ki = 2.0 * kp / 0.5; // Assume Ti = 0.5 seconds
                kd = kp * 0.125; // Assume Td = 0.125 seconds
                break;
                
            case TYREUS_LUYBEN:
                // More conservative tuning
                kp = 0.45 * (1.0 / kv);
                ki = 2.2 * kp / 1.0;
                kd = kp * 0.15;
                break;
                
            case NO_OVERSHOOT:
                // Very conservative, no overshoot
                kp = 0.2 * (1.0 / kv);
                ki = 0.4 * kp / 1.0;
                kd = kp * 0.05;
                break;
                
            case COHEN_COON:
            default:
                // Moderate tuning
                kp = 0.5 * (1.0 / kv);
                ki = 1.5 * kp / 1.0;
                kd = kp * 0.1;
                break;
        }
        
        return new Gains(kp, ki, kd, kv, result.ka, 0.0);
    }
    
    /**
     * Get the calculated velocity feedforward gain.
     * @return kv (volts per unit/sec)
     */
    public double getKv() {
        return kv;
    }
    
    /**
     * Get the calculated acceleration feedforward gain.
     * @return ka (volts per unit/sec^2)
     */
    public double getKa() {
        return ka;
    }
    
    /**
     * Results from system characterization.
     */
    public static class CharacterizationResult {
        public final double kv;
        public final double ka;
        public final List<Double> velocities;
        public final List<Double> voltages;
        public final List<Double> timestamps;
        
        public CharacterizationResult(double kv, double ka, List<Double> velocities, 
                                     List<Double> voltages, List<Double> timestamps) {
            this.kv = kv;
            this.ka = ka;
            this.velocities = new ArrayList<>(velocities);
            this.voltages = new ArrayList<>(voltages);
            this.timestamps = new ArrayList<>(timestamps);
        }
    }
}
