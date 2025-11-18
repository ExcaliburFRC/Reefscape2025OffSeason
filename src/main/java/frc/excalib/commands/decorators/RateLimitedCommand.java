package frc.excalib.commands.decorators;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A decorator that rate-limits command execution to a maximum frequency.
 * Useful for preventing system overload and ensuring consistent loop timing.
 * 
 * Example:
 * <pre>
 * Command rateLimited = new RateLimitedCommand(visionCommand, 20); // max 20 Hz
 * </pre>
 * 
 * @author Excalib Team
 */
public class RateLimitedCommand extends Command {
    private final Command command;
    private final double minPeriodSeconds;
    private double lastExecuteTime;

    /**
     * Creates a new RateLimitedCommand.
     * 
     * @param command the command to rate limit
     * @param maxFrequencyHz maximum execution frequency in Hz (must be positive)
     */
    public RateLimitedCommand(Command command, double maxFrequencyHz) {
        if (maxFrequencyHz <= 0) {
            throw new IllegalArgumentException("frequency must be positive");
        }
        this.command = command;
        this.minPeriodSeconds = 1.0 / maxFrequencyHz;
        this.lastExecuteTime = 0.0;
        
        // Inherit requirements from wrapped command
        m_requirements.addAll(command.getRequirements());
    }

    @Override
    public void initialize() {
        lastExecuteTime = 0.0;
        command.initialize();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Only execute if enough time has passed since last execution
        if (currentTime - lastExecuteTime >= minPeriodSeconds) {
            command.execute();
            lastExecuteTime = currentTime;
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    /**
     * Get the minimum period between executions.
     * @return period in seconds
     */
    public double getMinPeriod() {
        return minPeriodSeconds;
    }

    /**
     * Get the maximum execution frequency.
     * @return frequency in Hz
     */
    public double getMaxFrequency() {
        return 1.0 / minPeriodSeconds;
    }
}
