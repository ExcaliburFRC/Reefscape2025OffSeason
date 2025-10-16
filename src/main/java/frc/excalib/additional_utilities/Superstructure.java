package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.excalib.commands.CommandMutex;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Generic, extensible superstructure manager.
 * Controls subsystem coordination, automation, and parameterized actions.
 */
public abstract class Superstructure<G extends Enum<G>, P> {
    private final EnumMap<G, Function<P, Command>> goalCommands;
    private G currentGoal;
    private P currentParams;
    private CommandMutex commandMutex;

    protected Superstructure(Class<G> goalClass) {
        goalCommands = new EnumMap<>(goalClass);
    }

    /** Register a goal with a parameterized command factory. */
    protected void registerGoal(G goal, Function<P, Command> commandFactory) {
        goalCommands.put(goal, commandFactory);
    }

    /** Register a goal with no parameters. */
    protected void registerGoal(G goal, Supplier<Command> commandSupplier) {
        goalCommands.put(goal, p -> commandSupplier.get());
    }

    /** Schedule a goal with parameters. */
    public void setGoal(G goal, P params) {
        if (!goalCommands.containsKey(goal))
            throw new IllegalArgumentException("Goal not registered: " + goal);

        currentGoal = goal;
        currentParams = params;

        commandMutex.schedule(goalCommands.get(goal).apply(params));
    }

    /** Schedule a goal without parameters. */
    public void setGoal(G goal) {
        setGoal(goal, null);
    }

    public G getCurrentGoal() { return currentGoal; }

    public P getCurrentParams() { return currentParams; }

    public boolean isAtCommandGoal() {
        return commandMutex.isRunning();
    }

    public void cancel() {
        commandMutex.cancel();
    }

    /** Allows dynamic replacement of goals (useful for runtime logic). */
    public void updateGoalCommand(G goal, Function<P, Command> commandFactory) {
        goalCommands.put(goal, commandFactory);
    }

    /** For debugging or telemetry. */
    public Map<G, Function<P, Command>> getRegisteredGoals() {
        return Map.copyOf(goalCommands);
    }
}
