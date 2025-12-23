package frc.excalib.safety;

import frc.excalib.control.motor.controllers.Motor;

/**
 * Monitors motor current and stops the motor if it exceeds safe limits.
 * Prevents motor and mechanism damage from stalls or jams.
 * 
 * Example:
 * <pre>
 * SafetyManager.register(CurrentLimitGuard.forMotor(intakeMotor, 40.0));
 * </pre>
 * 
 * @author Excalib Team
 */
public class CurrentLimitGuard implements SafetyGuard {
    private final Motor motor;
    private final double currentLimit;
    private final double durationSeconds;
    private double exceedStartTime = -1.0;
    
    private CurrentLimitGuard(Motor motor, double currentLimit, double durationSeconds) {
        this.motor = motor;
        this.currentLimit = currentLimit;
        this.durationSeconds = durationSeconds;
    }
    
    /**
     * Create a current limit guard for a motor.
     * @param motor the motor to monitor
     * @param currentLimit current limit in amps
     * @return a new guard instance
     */
    public static CurrentLimitGuard forMotor(Motor motor, double currentLimit) {
        return new CurrentLimitGuard(motor, currentLimit, 0.5);
    }
    
    /**
     * Create a current limit guard with custom duration.
     * @param motor the motor to monitor
     * @param currentLimit current limit in amps
     * @param durationSeconds how long current can exceed before triggering
     * @return a new guard instance
     */
    public static CurrentLimitGuard forMotor(Motor motor, double currentLimit, double durationSeconds) {
        return new CurrentLimitGuard(motor, currentLimit, durationSeconds);
    }
    
    @Override
    public boolean check() {
        double current = motor.getCurrent();
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        if (current > currentLimit) {
            if (exceedStartTime < 0) {
                exceedStartTime = currentTime;
            } else if (currentTime - exceedStartTime > durationSeconds) {
                onTriggered();
                return false;
            }
        } else {
            exceedStartTime = -1.0;
        }
        
        return true;
    }
    
    @Override
    public String getDescription() {
        return String.format("Prevents motor damage by limiting current to %.1fA", currentLimit);
    }
    
    @Override
    public void onTriggered() {
        System.err.println(String.format(
            "[CURRENT LIMIT] Motor %d exceeded %.1fA limit (current: %.1fA). Stopping motor.",
            motor.getDeviceID(), currentLimit, motor.getCurrent()
        ));
        motor.stopMotor();
    }
    
    @Override
    public String getName() {
        return "CurrentLimitGuard[Motor " + motor.getDeviceID() + "]";
    }
}
