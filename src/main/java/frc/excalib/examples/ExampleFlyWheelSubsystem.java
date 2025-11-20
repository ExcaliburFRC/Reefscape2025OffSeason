package frc.excalib.examples;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.TalonFXMotorIO;
import frc.excalib.control.motor.controllers.TalonFXMotorIOReal;
import frc.excalib.control.motor.controllers.TalonFXMotorInputsAutoLogged;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Example subsystem demonstrating AdvantageKit integration with excalib FlyWheel mechanism.
 * 
 * This example shows best practices for:
 * - Using the IO pattern for hardware abstraction
 * - Logging motor telemetry with AdvantageKit
 * - Supporting both real hardware, simulation, and replay
 * - Integrating with excalib mechanisms
 */
public class ExampleFlyWheelSubsystem extends SubsystemBase {
    private final TalonFXMotorIO motorIO;
    private final TalonFXMotorInputsAutoLogged inputs;
    private final FlyWheel flywheel;
    
    /**
     * Creates a new ExampleFlyWheelSubsystem.
     * 
     * @param motorIO The motor IO implementation (real hardware or simulation)
     * @param flywheel The FlyWheel mechanism for velocity control
     */
    public ExampleFlyWheelSubsystem(TalonFXMotorIO motorIO, FlyWheel flywheel) {
        this.motorIO = motorIO;
        this.inputs = new TalonFXMotorInputsAutoLogged();
        this.flywheel = flywheel;
    }
    
    /**
     * Factory method to create this subsystem with real hardware.
     * 
     * @param canId The CAN ID of the motor
     * @param gains PID and feedforward gains for the flywheel
     * @return A new ExampleFlyWheelSubsystem configured for real hardware
     */
    public static ExampleFlyWheelSubsystem createReal(int canId, Gains gains) {
        TalonFXMotorIOReal motorIOReal = new TalonFXMotorIOReal(canId);
        FlyWheel flywheel = new FlyWheel(
            motorIOReal.getMotor(),
            100.0,  // maxAcceleration (rad/s^2)
            1000.0, // maxJerk (rad/s^3)
            gains
        );
        return new ExampleFlyWheelSubsystem(motorIOReal, flywheel);
    }
    
    /**
     * Factory method to create this subsystem for simulation/replay.
     * 
     * @param gains PID and feedforward gains for the flywheel
     * @return A new ExampleFlyWheelSubsystem configured for simulation
     */
    public static ExampleFlyWheelSubsystem createSim(Gains gains) {
        // For simulation, we can't use the real FlyWheel, so we just use the IO pattern
        return new ExampleFlyWheelSubsystem(new TalonFXMotorIO(), null);
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware and log them
        motorIO.updateInputs(inputs);
        Logger.processInputs("ExampleFlyWheel", inputs);
        
        // Log additional computed values
        Logger.recordOutput("ExampleFlyWheel/AtSetpoint", isAtSetpoint());
    }
    
    /**
     * Command to run the flywheel at a target velocity using simple feedforward control.
     * 
     * @param velocityRPS Target velocity in rotations per second
     * @return Command that runs the flywheel at the target velocity
     */
    public Command runVelocity(DoubleSupplier velocityRPS) {
        return run(() -> {
            double targetVel = velocityRPS.getAsDouble();
            Logger.recordOutput("ExampleFlyWheel/TargetVelocityRPS", targetVel);
            
            // Simple feedforward control: voltage = kV * velocity
            // Typical kV for a flywheel is around 0.12 V/(rad/s) or 0.02 V/RPS
            double kV = 0.02; // Volts per rotation per second
            double feedforwardVoltage = kV * targetVel;
            
            // Clamp to motor voltage limits
            feedforwardVoltage = Math.max(-12.0, Math.min(12.0, feedforwardVoltage));
            
            motorIO.setVoltage(feedforwardVoltage);
        });
    }
    
    /**
     * Sets the motor voltage directly.
     * 
     * @param volts The voltage to apply (-12.0 to 12.0)
     */
    public void setVoltage(double volts) {
        motorIO.setVoltage(volts);
        Logger.recordOutput("ExampleFlyWheel/CommandedVoltage", volts);
    }
    
    /**
     * Stops the motor.
     */
    public void stop() {
        motorIO.stop();
    }
    
    /**
     * Gets the current motor velocity.
     * 
     * @return The velocity in rotations per second
     */
    public double getVelocityRPS() {
        return inputs.velocityRotationsPerSecond;
    }
    
    /**
     * Gets the current draw of the motor.
     * 
     * @return The current in amps
     */
    public double getCurrentAmps() {
        return inputs.currentAmps;
    }
    
    /**
     * Checks if the flywheel is at the target setpoint (within tolerance).
     * 
     * @return True if at setpoint
     */
    public boolean isAtSetpoint() {
        // Simple check - in real implementation would compare with target
        return Math.abs(inputs.velocityRotationsPerSecond) > 0.1;
    }
    
    /**
     * Command to stop the flywheel.
     * 
     * @return Command that stops the flywheel
     */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
