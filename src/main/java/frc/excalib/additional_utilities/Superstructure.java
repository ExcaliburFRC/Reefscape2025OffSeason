package frc.excalib.additional_utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.commands.CommandMutex;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Generic superstructure manager with proper WPILib Trigger support.
 */
public abstract class Superstructure<G extends Enum<G>, P> {
    private final EnumMap<G, Function<P, Command>> goalCommands;
    private G currentGoal;
    private P currentParams;
    private final CommandMutex commandMutex;

    private boolean goalStartedFlag = false;
    private boolean goalCompletedFlag = false;

    private final Trigger goalStartedTrigger = new Trigger(() -> {
        if (goalStartedFlag) {
            goalStartedFlag = false;
            return true;
        }
        return false;
    });

    private final Trigger goalCompletedTrigger = new Trigger(() -> {
        if (goalCompletedFlag) {
            goalCompletedFlag = false;
            return true;
        }
        return false;
    });

    protected Superstructure(Class<G> goalClass) {
        goalCommands = new EnumMap<>(goalClass);
        commandMutex = new CommandMutex();
    }

    /** Register a goal with parameters. */
    protected void registerGoal(G goal, Function<P, Command> commandFactory) {
        goalCommands.put(goal, commandFactory);
    }

    /** Register a goal without parameters. */
    protected void registerGoal(G goal, Supplier<Command> commandSupplier) {
        goalCommands.put(goal, p -> commandSupplier.get());
    }

    /** Schedule a goal with parameters. Triggers fire automatically. */
    public void setGoal(G goal, P params) {
        if (!goalCommands.containsKey(goal))
            throw new IllegalArgumentException("Goal not registered: " + goal);

        currentGoal = goal;
        currentParams = params;

        goalStartedFlag = true;

        Command command = goalCommands.get(goal).apply(params);
        command.finallyDo(() -> goalCompletedFlag = true);

        commandMutex.schedule(command);
    }

    /** Schedule a goal without parameters. */
    public void setGoal(G goal) {
        setGoal(goal, null);
    }

    public G getCurrentGoal() { return currentGoal; }
    public P getCurrentParams() { return currentParams; }

    public boolean isGoalComplete() { return !commandMutex.isRunning(); }

    public void cancel() { commandMutex.cancel(); }

    public void updateGoalCommand(G goal, Function<P, Command> commandFactory) {
        goalCommands.put(goal, commandFactory);
    }

    public Map<G, Function<P, Command>> getRegisteredGoals() {
        return Map.copyOf(goalCommands);
    }

    /** Expose triggers for external command bindings. */
    public Trigger onGoalStart() { return goalStartedTrigger; }
    public Trigger onGoalComplete() { return goalCompletedTrigger; }
}
