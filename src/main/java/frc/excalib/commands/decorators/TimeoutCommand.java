package frc.excalib.commands.decorators;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A decorator that adds a timeout to any command.
 * The command will be cancelled if it doesn't finish within the timeout period.
 * 
 * Example:
 * <pre>
 * Command timedIntake = new TimeoutCommand(intakeCommand, 3.0); // 3 seconds max
 * </pre>
 * 
 * @author Excalib Team
 */
public class TimeoutCommand extends Command {
    private final Command command;
    private final double timeoutSeconds;
    private final Timer timer;
    private boolean timedOut;

    /**
     * Creates a new TimeoutCommand.
     * 
     * @param command the command to wrap
     * @param timeoutSeconds timeout in seconds (must be positive)
     */
    public TimeoutCommand(Command command, double timeoutSeconds) {
        if (timeoutSeconds <= 0) {
            throw new IllegalArgumentException("timeout must be positive");
        }
        this.command = command;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new Timer();
        this.timedOut = false;
        
        // Inherit requirements from wrapped command
        m_requirements.addAll(command.getRequirements());
    }

    @Override
    public void initialize() {
        timer.restart();
        timedOut = false;
        command.initialize();
    }

    @Override
    public void execute() {
        if (!timedOut) {
            command.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeoutSeconds)) {
            timedOut = true;
            return true;
        }
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted || timedOut);
        timer.stop();
    }

    /**
     * Check if the command timed out.
     * @return true if the command was cancelled due to timeout
     */
    public boolean hasTimedOut() {
        return timedOut;
    }

    /**
     * Get the timeout duration.
     * @return timeout in seconds
     */
    public double getTimeoutSeconds() {
        return timeoutSeconds;
    }

    /**
     * Get the elapsed time.
     * @return elapsed time in seconds
     */
    public double getElapsedTime() {
        return timer.get();
    }
}
