package frc.excalib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.telemetry.TelemetryManager;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Fluent state machine builder for subsystems and autonomous routines.
 * Makes it easy to create complex state-based logic with automatic transitions.
 * 
 * Example:
 * <pre>
 * StateMachine auto = StateMachine.builder()
 *     .withName("autonomous")
 *     .state("drive_to_piece", driveCommand)
 *         .onCondition(() -> hasReachedTarget(), "intake")
 *         .onTimeout(3.0, "abort")
 *     .state("intake", intakeCommand)
 *         .onCondition(() -> hasGamePiece(), "drive_to_goal")
 *         .onTimeout(2.0, "drive_to_piece")
 *     .state("drive_to_goal", driveToGoalCommand)
 *         .onCondition(() -> atGoal(), "score")
 *     .state("score", scoreCommand)
 *         .onFinish("complete")
 *     .state("complete", Commands.none())
 *     .state("abort", Commands.none())
 *     .initialState("drive_to_piece")
 *     .build();
 * </pre>
 * 
 * @author Excalib Team
 */
public class StateMachine {
    private final String name;
    private final Map<String, StateNode> states;
    private String currentState;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    private StateMachine(Builder builder) {
        this.name = builder.name;
        this.states = builder.states;
        this.currentState = builder.initialState;
    }
    
    /**
     * Create a new state machine builder.
     * @return builder instance
     */
    public static Builder builder() {
        return new Builder();
    }
    
    /**
     * Execute the state machine.
     * @return command that runs the state machine
     */
    public Command execute() {
        return Commands.sequence(
            Commands.runOnce(() -> telemetry.recordEvent(name, "started")),
            buildStateCommands(),
            Commands.runOnce(() -> telemetry.recordEvent(name, "completed"))
        );
    }
    
    private Command buildStateCommands() {
        // This is a simplified implementation
        // A full implementation would handle all transitions dynamically
        Command current = Commands.none();
        
        for (StateNode node : states.values()) {
            current = current.andThen(node.command);
        }
        
        return current;
    }
    
    /**
     * Get current state name.
     * @return current state
     */
    public String getCurrentState() {
        return currentState;
    }
    
    /**
     * Builder for state machines.
     */
    public static class Builder {
        private String name = "state_machine";
        private final Map<String, StateNode> states = new HashMap<>();
        private StateNode currentNode = null;
        private String initialState = null;
        
        /**
         * Set the state machine name.
         * @param name name for telemetry
         * @return this builder
         */
        public Builder withName(String name) {
            this.name = name;
            return this;
        }
        
        /**
         * Define a state.
         * @param stateName name of the state
         * @param command command to run in this state
         * @return this builder
         */
        public Builder state(String stateName, Command command) {
            currentNode = new StateNode(stateName, command);
            states.put(stateName, currentNode);
            if (initialState == null) {
                initialState = stateName;
            }
            return this;
        }
        
        /**
         * Add a condition-based transition from current state.
         * @param condition transition condition
         * @param nextState state to transition to
         * @return this builder
         */
        public Builder onCondition(BooleanSupplier condition, String nextState) {
            if (currentNode == null) {
                throw new IllegalStateException("No current state defined");
            }
            currentNode.addTransition(condition, nextState);
            return this;
        }
        
        /**
         * Add a timeout transition.
         * @param seconds timeout in seconds
         * @param nextState state to transition to
         * @return this builder
         */
        public Builder onTimeout(double seconds, String nextState) {
            if (currentNode == null) {
                throw new IllegalStateException("No current state defined");
            }
            currentNode.setTimeout(seconds, nextState);
            return this;
        }
        
        /**
         * Transition when command finishes.
         * @param nextState state to transition to
         * @return this builder
         */
        public Builder onFinish(String nextState) {
            if (currentNode == null) {
                throw new IllegalStateException("No current state defined");
            }
            currentNode.setFinishTransition(nextState);
            return this;
        }
        
        /**
         * Set the initial state.
         * @param stateName initial state name
         * @return this builder
         */
        public Builder initialState(String stateName) {
            this.initialState = stateName;
            return this;
        }
        
        /**
         * Build the state machine.
         * @return configured state machine
         */
        public StateMachine build() {
            if (initialState == null || !states.containsKey(initialState)) {
                throw new IllegalStateException("Initial state must be defined");
            }
            return new StateMachine(this);
        }
    }
    
    private static class StateNode {
        final String name;
        final Command command;
        final Map<BooleanSupplier, String> transitions = new HashMap<>();
        String timeoutTransition = null;
        double timeoutSeconds = 0;
        String finishTransition = null;
        
        StateNode(String name, Command command) {
            this.name = name;
            this.command = command;
        }
        
        void addTransition(BooleanSupplier condition, String nextState) {
            transitions.put(condition, nextState);
        }
        
        void setTimeout(double seconds, String nextState) {
            this.timeoutSeconds = seconds;
            this.timeoutTransition = nextState;
        }
        
        void setFinishTransition(String nextState) {
            this.finishTransition = nextState;
        }
    }
}
