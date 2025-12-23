package frc.excalib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Factory for creating common command patterns with minimal code.
 * Eliminates repetitive command creation boilerplate.
 * 
 * @author Excalib Team
 */
public class CommandFactory {
    
    /**
     * Create a command that runs until a condition is met.
     * 
     * @param subsystem subsystem requirement
     * @param action action to run
     * @param condition end condition
     * @return command
     */
    public static Command runUntil(SubsystemBase subsystem, Runnable action, BooleanSupplier condition) {
        return Commands.run(action, subsystem).until(condition);
    }
    
    /**
     * Create a command that runs while a condition is true.
     * 
     * @param subsystem subsystem requirement
     * @param action action to run
     * @param condition continue condition
     * @return command
     */
    public static Command runWhile(SubsystemBase subsystem, Runnable action, BooleanSupplier condition) {
        return Commands.run(action, subsystem).onlyWhile(condition);
    }
    
    /**
     * Create a command that waits for a condition with timeout.
     * 
     * @param condition condition to wait for
     * @param timeoutSeconds timeout in seconds
     * @return command
     */
    public static Command waitForCondition(BooleanSupplier condition, double timeoutSeconds) {
        return Commands.waitUntil(condition).withTimeout(timeoutSeconds);
    }
    
    /**
     * Create a position control command.
     * 
     * @param subsystem subsystem requirement
     * @param setPosition action to set position
     * @param getCurrentPosition position supplier
     * @param targetPosition target position
     * @param tolerance position tolerance
     * @return command
     */
    public static Command positionControl(
            SubsystemBase subsystem,
            Runnable setPosition,
            DoubleSupplier getCurrentPosition,
            double targetPosition,
            double tolerance) {
        return Commands.run(setPosition, subsystem)
            .until(() -> Math.abs(getCurrentPosition.getAsDouble() - targetPosition) < tolerance);
    }
    
    /**
     * Create a velocity control command.
     * 
     * @param subsystem subsystem requirement
     * @param setVelocity action to set velocity
     * @param getCurrentVelocity velocity supplier
     * @param targetVelocity target velocity
     * @param tolerance velocity tolerance
     * @return command
     */
    public static Command velocityControl(
            SubsystemBase subsystem,
            Runnable setVelocity,
            DoubleSupplier getCurrentVelocity,
            double targetVelocity,
            double tolerance) {
        return Commands.run(setVelocity, subsystem)
            .until(() -> Math.abs(getCurrentVelocity.getAsDouble() - targetVelocity) < tolerance);
    }
    
    /**
     * Create a ramp command that gradually changes a value.
     * 
     * @param subsystem subsystem requirement
     * @param setValue consumer that sets the value
     * @param startValue starting value
     * @param endValue ending value
     * @param duration duration in seconds
     * @return command
     */
    public static Command ramp(
            SubsystemBase subsystem,
            java.util.function.DoubleConsumer setValue,
            double startValue,
            double endValue,
            double duration) {
        return Commands.run(() -> {
            double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            double progress = Math.min(1.0, elapsed / duration);
            double value = startValue + (endValue - startValue) * progress;
            setValue.accept(value);
        }, subsystem).withTimeout(duration);
    }
    
    /**
     * Create a toggle command that switches between two states.
     * 
     * @param state1 first state command
     * @param state2 second state command
     * @param currentState supplier for current state (true = state1, false = state2)
     * @return toggle command
     */
    public static Command toggle(Command state1, Command state2, BooleanSupplier currentState) {
        return Commands.either(state2, state1, currentState);
    }
    
    /**
     * Create a sequence with automatic delays between steps.
     * 
     * @param delaySeconds delay between each command
     * @param commands commands to run
     * @return sequenced command with delays
     */
    public static Command sequenceWithDelay(double delaySeconds, Command... commands) {
        Command result = Commands.none();
        for (Command cmd : commands) {
            result = result.andThen(cmd).andThen(Commands.waitSeconds(delaySeconds));
        }
        return result;
    }
    
    /**
     * Create a command that runs multiple commands in parallel and waits for all.
     * 
     * @param commands commands to run
     * @return parallel command group
     */
    public static Command parallelAll(Command... commands) {
        return Commands.parallel(commands);
    }
    
    /**
     * Create a command that runs multiple commands in parallel and ends when any finishes.
     * 
     * @param commands commands to run
     * @return parallel race group
     */
    public static Command parallelRace(Command... commands) {
        return Commands.race(commands);
    }
    
    /**
     * Create a repeating command that runs N times.
     * 
     * @param command command to repeat
     * @param count number of times to repeat
     * @return repeating command
     */
    public static Command repeat(Command command, int count) {
        Command result = Commands.none();
        for (int i = 0; i < count; i++) {
            result = result.andThen(command);
        }
        return result;
    }
    
    /**
     * Create a command that prints debug info.
     * 
     * @param message message to print
     * @return print command
     */
    public static Command print(String message) {
        return Commands.runOnce(() -> System.out.println("[DEBUG] " + message));
    }
}
