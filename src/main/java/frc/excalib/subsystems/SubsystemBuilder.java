package frc.excalib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.telemetry.TelemetryManager;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Fluent builder for creating subsystems with automatic telemetry, state management,
 * and command generation. Eliminates boilerplate and makes subsystem creation fast.
 * 
 * Example:
 * <pre>
 * public class IntakeSubsystem extends SubsystemBase {
 *     private final SubsystemBuilder builder;
 *     
 *     public IntakeSubsystem() {
 *         builder = SubsystemBuilder.create(this, "intake")
 *             .withState("idle", () -> motor.stop())
 *             .withState("intaking", () -> motor.setPercentage(0.8))
 *             .withState("outtaking", () -> motor.setPercentage(-0.5))
 *             .withTrigger("has_piece", sensor::hasGamePiece)
 *             .withAutoTelemetry()
 *             .build();
 *     }
 *     
 *     public Command intakeCommand() {
 *         return builder.commandForState("intaking");
 *     }
 * }
 * </pre>
 * 
 * @author Excalib Team
 */
public class SubsystemBuilder {
    private final SubsystemBase subsystem;
    private final String name;
    private final Map<String, Runnable> states = new HashMap<>();
    private final Map<String, BooleanSupplier> triggers = new HashMap<>();
    private final List<String> telemetryKeys = new ArrayList<>();
    private boolean autoTelemetry = false;
    private String defaultState = null;
    
    private SubsystemBuilder(SubsystemBase subsystem, String name) {
        this.subsystem = subsystem;
        this.name = name;
    }
    
    /**
     * Create a new SubsystemBuilder.
     * @param subsystem the subsystem being built
     * @param name subsystem name for telemetry
     * @return a new builder instance
     */
    public static SubsystemBuilder create(SubsystemBase subsystem, String name) {
        return new SubsystemBuilder(subsystem, name);
    }
    
    /**
     * Define a state with its associated action.
     * @param stateName name of the state
     * @param action action to execute in this state
     * @return this for chaining
     */
    public SubsystemBuilder withState(String stateName, Runnable action) {
        states.put(stateName, action);
        if (defaultState == null) {
            defaultState = stateName;
        }
        return this;
    }
    
    /**
     * Define a trigger (boolean condition).
     * @param triggerName name of the trigger
     * @param condition condition supplier
     * @return this for chaining
     */
    public SubsystemBuilder withTrigger(String triggerName, BooleanSupplier condition) {
        triggers.put(triggerName, condition);
        return this;
    }
    
    /**
     * Add a telemetry key to track.
     * @param key telemetry key
     * @return this for chaining
     */
    public SubsystemBuilder withTelemetry(String key) {
        telemetryKeys.add(key);
        return this;
    }
    
    /**
     * Enable automatic telemetry for all states and triggers.
     * @return this for chaining
     */
    public SubsystemBuilder withAutoTelemetry() {
        this.autoTelemetry = true;
        return this;
    }
    
    /**
     * Set the default state.
     * @param stateName name of the default state
     * @return this for chaining
     */
    public SubsystemBuilder withDefaultState(String stateName) {
        this.defaultState = stateName;
        return this;
    }
    
    /**
     * Build and return the configured builder.
     * @return this builder
     */
    public SubsystemBuilder build() {
        // Set default command if default state is set
        if (defaultState != null && states.containsKey(defaultState)) {
            subsystem.setDefaultCommand(commandForState(defaultState));
        }
        return this;
    }
    
    /**
     * Create a command that runs a specific state.
     * @param stateName name of the state
     * @return command for this state
     */
    public Command commandForState(String stateName) {
        Runnable action = states.get(stateName);
        if (action == null) {
            throw new IllegalArgumentException("Unknown state: " + stateName);
        }
        
        return subsystem.runOnce(() -> {
            if (autoTelemetry) {
                TelemetryManager.getInstance().recordString(name + "/state", stateName);
            }
            action.run();
        }).withName(name + "_" + stateName);
    }
    
    /**
     * Get a trigger by name.
     * @param triggerName name of the trigger
     * @return the trigger
     */
    public Trigger trigger(String triggerName) {
        BooleanSupplier condition = triggers.get(triggerName);
        if (condition == null) {
            throw new IllegalArgumentException("Unknown trigger: " + triggerName);
        }
        return new Trigger(condition);
    }
    
    /**
     * Execute an action in a state.
     * @param stateName name of the state
     */
    public void setState(String stateName) {
        Runnable action = states.get(stateName);
        if (action != null) {
            action.run();
            if (autoTelemetry) {
                TelemetryManager.getInstance().recordString(name + "/current_state", stateName);
            }
        }
    }
    
    /**
     * Update telemetry (call in periodic).
     */
    public void updateTelemetry() {
        if (!autoTelemetry) return;
        
        // Record all trigger states
        for (Map.Entry<String, BooleanSupplier> entry : triggers.entrySet()) {
            TelemetryManager.getInstance().recordBoolean(
                name + "/" + entry.getKey(),
                entry.getValue().getAsBoolean()
            );
        }
    }
    
    /**
     * Get subsystem name.
     * @return subsystem name
     */
    public String getName() {
        return name;
    }
}
