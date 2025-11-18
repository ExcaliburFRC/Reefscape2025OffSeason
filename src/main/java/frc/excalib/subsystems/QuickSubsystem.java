package frc.excalib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Ultra-fast subsystem creation with zero boilerplate.
 * Perfect for simple subsystems that just need states and commands.
 * 
 * Example - create a complete intake subsystem in 10 lines:
 * <pre>
 * public class IntakeSubsystem extends QuickSubsystem {
 *     public IntakeSubsystem(Motor motor, DigitalInput sensor) {
 *         super("intake");
 *         
 *         addState("idle", motor::stopMotor);
 *         addState("intaking", () -> motor.setPercentage(0.8));
 *         addState("outtaking", () -> motor.setPercentage(-0.5));
 *         
 *         addSensor("has_piece", sensor::get);
 *         addSensor("current", motor::getCurrent);
 *         
 *         setDefaultState("idle");
 *     }
 * }
 * 
 * // Usage
 * intake.run("intaking");
 * if (intake.getBool("has_piece")) { ... }
 * </pre>
 * 
 * @author Excalib Team
 */
public abstract class QuickSubsystem extends SubsystemBase {
    private final String name;
    private final Map<String, Runnable> states = new HashMap<>();
    private final Map<String, DoubleSupplier> sensors = new HashMap<>();
    private String currentState = "idle";
    
    /**
     * Create a quick subsystem.
     * @param name subsystem name
     */
    protected QuickSubsystem(String name) {
        this.name = name;
        setName(name);
    }
    
    /**
     * Add a state.
     * @param stateName state name
     * @param action action to execute
     */
    protected void addState(String stateName, Runnable action) {
        states.put(stateName, action);
    }
    
    /**
     * Add a sensor/value supplier.
     * @param sensorName sensor name
     * @param supplier value supplier
     */
    protected void addSensor(String sensorName, DoubleSupplier supplier) {
        sensors.put(sensorName, supplier);
    }
    
    /**
     * Set default state.
     * @param stateName state name
     */
    protected void setDefaultState(String stateName) {
        if (!states.containsKey(stateName)) {
            throw new IllegalArgumentException("Unknown state: " + stateName);
        }
        setDefaultCommand(commandFor(stateName));
    }
    
    /**
     * Run a state immediately.
     * @param stateName state to run
     */
    public void run(String stateName) {
        Runnable action = states.get(stateName);
        if (action != null) {
            currentState = stateName;
            action.run();
        }
    }
    
    /**
     * Get a command for a state.
     * @param stateName state name
     * @return command
     */
    public Command commandFor(String stateName) {
        return runOnce(() -> run(stateName)).withName(name + "_" + stateName);
    }
    
    /**
     * Get sensor value.
     * @param sensorName sensor name
     * @return sensor value
     */
    public double get(String sensorName) {
        DoubleSupplier supplier = sensors.get(sensorName);
        return supplier != null ? supplier.getAsDouble() : 0.0;
    }
    
    /**
     * Get sensor as boolean (> 0.5).
     * @param sensorName sensor name
     * @return true if > 0.5
     */
    public boolean getBool(String sensorName) {
        return get(sensorName) > 0.5;
    }
    
    /**
     * Get current state.
     * @return current state name
     */
    public String getCurrentState() {
        return currentState;
    }
    
    @Override
    public void periodic() {
        // Override in subclass if needed
    }
}
