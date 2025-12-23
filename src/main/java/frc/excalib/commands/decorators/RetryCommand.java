package frc.excalib.commands.decorators;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A decorator that retries a command a specified number of times if it fails.
 * Useful for making autonomous sequences more robust.
 * 
 * Example:
 * <pre>
 * Command robustIntake = new RetryCommand(intakeCommand, 3);
 * </pre>
 * 
 * @author Excalib Team
 */
public class RetryCommand extends Command {
    private final Command command;
    private final int maxAttempts;
    private int currentAttempt;
    private boolean commandScheduled;

    /**
     * Creates a new RetryCommand.
     * 
     * @param command the command to retry
     * @param maxAttempts maximum number of attempts (must be >= 1)
     */
    public RetryCommand(Command command, int maxAttempts) {
        if (maxAttempts < 1) {
            throw new IllegalArgumentException("maxAttempts must be at least 1");
        }
        this.command = command;
        this.maxAttempts = maxAttempts;
        this.currentAttempt = 0;
        this.commandScheduled = false;
        
        // Inherit requirements from wrapped command
        m_requirements.addAll(command.getRequirements());
    }

    @Override
    public void initialize() {
        currentAttempt = 1;
        commandScheduled = false;
        scheduleCommand();
    }

    @Override
    public void execute() {
        // Check if the command finished unsuccessfully
        if (!command.isScheduled() && !commandScheduled) {
            return; // Command was never scheduled, shouldn't happen
        }
        
        if (!command.isScheduled()) {
            // Command finished, check if we should retry
            if (currentAttempt < maxAttempts) {
                currentAttempt++;
                scheduleCommand();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finished if command succeeded or we've exhausted retries
        return !command.isScheduled() && 
               (commandScheduled && currentAttempt >= maxAttempts);
    }

    @Override
    public void end(boolean interrupted) {
        if (command.isScheduled()) {
            command.cancel();
        }
    }

    private void scheduleCommand() {
        command.initialize();
        commandScheduled = true;
    }

    /**
     * Get the current attempt number.
     * @return current attempt (1-indexed)
     */
    public int getCurrentAttempt() {
        return currentAttempt;
    }

    /**
     * Get the maximum number of attempts.
     * @return max attempts
     */
    public int getMaxAttempts() {
        return maxAttempts;
    }
}
