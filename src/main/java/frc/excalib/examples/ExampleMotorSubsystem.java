package frc.excalib.examples;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotorIO;
import frc.excalib.control.motor.controllers.TalonFXMotorIOReal;
import frc.excalib.control.motor.controllers.TalonFXMotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Example subsystem demonstrating AdvantageKit integration with excalib motor controllers.
 * 
 * This example shows best practices for:
 * - Using the IO pattern for hardware abstraction
 * - Logging motor telemetry with AdvantageKit
 * - Supporting both real hardware and simulation
 */
public class ExampleMotorSubsystem extends SubsystemBase {
    private final TalonFXMotorIO motorIO;
    private final TalonFXMotorInputsAutoLogged inputs;
    
    /**
     * Creates a new ExampleMotorSubsystem.
     * 
     * @param motorIO The motor IO implementation (real hardware or simulation)
     */
    public ExampleMotorSubsystem(TalonFXMotorIO motorIO) {
        this.motorIO = motorIO;
        this.inputs = new TalonFXMotorInputsAutoLogged();
    }
    
    /**
     * Factory method to create this subsystem with real hardware.
     * 
     * @param canId The CAN ID of the motor
     * @return A new ExampleMotorSubsystem configured for real hardware
     */
    public static ExampleMotorSubsystem createReal(int canId) {
        return new ExampleMotorSubsystem(new TalonFXMotorIOReal(canId));
    }
    
    /**
     * Factory method to create this subsystem for simulation/replay.
     * 
     * @return A new ExampleMotorSubsystem configured for simulation
     */
    public static ExampleMotorSubsystem createSim() {
        return new ExampleMotorSubsystem(new TalonFXMotorIO());
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware and log them
        motorIO.updateInputs(inputs);
        Logger.processInputs("ExampleMotor", inputs);
    }
    
    /**
     * Sets the motor output voltage.
     * 
     * @param volts The voltage to apply (-12.0 to 12.0)
     */
    public void setVoltage(double volts) {
        motorIO.setVoltage(volts);
    }
    
    /**
     * Sets the motor output as a percentage.
     * 
     * @param percentage The percentage to apply (-1.0 to 1.0)
     */
    public void setPercentage(double percentage) {
        motorIO.setPercentage(percentage);
    }
    
    /**
     * Stops the motor.
     */
    public void stop() {
        motorIO.stop();
    }
    
    /**
     * Gets the current motor position.
     * 
     * @return The position in rotations
     */
    public double getPosition() {
        return inputs.positionRotations;
    }
    
    /**
     * Gets the current motor velocity.
     * 
     * @return The velocity in rotations per second
     */
    public double getVelocity() {
        return inputs.velocityRotationsPerSecond;
    }
    
    /**
     * Gets the current draw of the motor.
     * 
     * @return The current in amps
     */
    public double getCurrent() {
        return inputs.currentAmps;
    }
}
