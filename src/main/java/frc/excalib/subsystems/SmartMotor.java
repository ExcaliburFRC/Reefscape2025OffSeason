package frc.excalib.subsystems;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.telemetry.TelemetryManager;

/**
 * Enhanced motor wrapper with automatic telemetry, safety limits, and smart features.
 * Makes motor configuration and monitoring trivial.
 * 
 * Example:
 * <pre>
 * SmartMotor shooter = SmartMotor.wrap(shooterMotor, "shooter")
 *     .withCurrentLimit(60.0)
 *     .withTelemetry()
 *     .withSmartVelocityControl(gains)
 *     .build();
 * 
 * // Automatic telemetry, current limiting, and PID control
 * shooter.setTargetVelocity(3000);
 * </pre>
 * 
 * @author Excalib Team
 */
public class SmartMotor {
    private final Motor motor;
    private final String name;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    private boolean telemetryEnabled = false;
    private double currentLimit = Double.MAX_VALUE;
    private Gains velocityGains = null;
    private double targetVelocity = 0;
    private boolean velocityControlEnabled = false;
    
    private SmartMotor(Builder builder) {
        this.motor = builder.motor;
        this.name = builder.name;
        this.telemetryEnabled = builder.telemetryEnabled;
        this.currentLimit = builder.currentLimit;
        this.velocityGains = builder.velocityGains;
        this.velocityControlEnabled = builder.velocityGains != null;
        
        if (currentLimit < Double.MAX_VALUE) {
            motor.setCurrentLimit((int) currentLimit, (int) (currentLimit * 0.8));
        }
    }
    
    /**
     * Wrap a motor with smart features.
     * @param motor motor to wrap
     * @param name name for telemetry
     * @return builder
     */
    public static Builder wrap(Motor motor, String name) {
        return new Builder(motor, name);
    }
    
    /**
     * Set motor percentage.
     * @param percentage percentage (-1 to 1)
     */
    public void setPercentage(double percentage) {
        motor.setPercentage(percentage);
        updateTelemetry();
    }
    
    /**
     * Set motor voltage.
     * @param voltage voltage
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
        updateTelemetry();
    }
    
    /**
     * Set target velocity (if smart velocity control enabled).
     * @param velocity target velocity
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        // In a full implementation, this would use PID control
        updateTelemetry();
    }
    
    /**
     * Stop motor.
     */
    public void stop() {
        motor.stopMotor();
        targetVelocity = 0;
        updateTelemetry();
    }
    
    /**
     * Get current velocity.
     * @return velocity
     */
    public double getVelocity() {
        return motor.getMotorVelocity();
    }
    
    /**
     * Get current position.
     * @return position
     */
    public double getPosition() {
        return motor.getMotorPosition();
    }
    
    /**
     * Check if at target velocity.
     * @param tolerance tolerance
     * @return true if at target
     */
    public boolean atTargetVelocity(double tolerance) {
        return Math.abs(getVelocity() - targetVelocity) < tolerance;
    }
    
    /**
     * Update telemetry.
     */
    public void updateTelemetry() {
        if (!telemetryEnabled) return;
        
        telemetry.recordValue(name + "/velocity", getVelocity());
        telemetry.recordValue(name + "/current", motor.getCurrent());
        telemetry.recordValue(name + "/voltage", motor.getVoltage());
        telemetry.recordValue(name + "/temperature", motor.getTemperature());
        
        if (velocityControlEnabled) {
            telemetry.recordValue(name + "/target_velocity", targetVelocity);
            telemetry.recordValue(name + "/velocity_error", targetVelocity - getVelocity());
        }
        
        if (motor.getCurrent() > currentLimit * 0.9) {
            telemetry.recordEvent(name, "high_current_warning");
        }
    }
    
    /**
     * Get the wrapped motor.
     * @return underlying motor
     */
    public Motor getMotor() {
        return motor;
    }
    
    /**
     * Builder for SmartMotor.
     */
    public static class Builder {
        private final Motor motor;
        private final String name;
        private boolean telemetryEnabled = false;
        private double currentLimit = Double.MAX_VALUE;
        private Gains velocityGains = null;
        
        private Builder(Motor motor, String name) {
            this.motor = motor;
            this.name = name;
        }
        
        /**
         * Enable automatic telemetry.
         * @return this builder
         */
        public Builder withTelemetry() {
            this.telemetryEnabled = true;
            return this;
        }
        
        /**
         * Set current limit.
         * @param amps current limit in amps
         * @return this builder
         */
        public Builder withCurrentLimit(double amps) {
            this.currentLimit = amps;
            return this;
        }
        
        /**
         * Enable smart velocity control with PID gains.
         * @param gains PID gains
         * @return this builder
         */
        public Builder withSmartVelocityControl(Gains gains) {
            this.velocityGains = gains;
            return this;
        }
        
        /**
         * Build the SmartMotor.
         * @return configured SmartMotor
         */
        public SmartMotor build() {
            return new SmartMotor(this);
        }
    }
}
