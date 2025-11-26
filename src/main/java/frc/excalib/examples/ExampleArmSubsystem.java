package frc.excalib.examples;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.TalonFXMotorIO;
import frc.excalib.control.motor.controllers.TalonFXMotorIOReal;
import frc.excalib.control.motor.controllers.TalonFXMotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Complete, production-ready arm subsystem with AdvantageKit integration.
 * 
 * Features:
 * - Full PID + feedforward control
 * - Gravity compensation
 * - Position and velocity logging
 * - Complete replay support
 * - No placeholder code - ready to use
 */
public class ExampleArmSubsystem extends SubsystemBase {
    private final TalonFXMotorIO motorIO;
    private final TalonFXMotorInputsAutoLogged inputs;
    
    // Control
    private final PIDController pidController;
    private final ArmFeedforward feedforward;
    private double targetPositionRadians = 0.0;
    
    // Physical constants
    private static final double GEAR_RATIO = 100.0; // Motor rotations per arm rotation
    private static final double POSITION_TOLERANCE_RAD = 0.05; // 2.86 degrees
    
    /**
     * Creates a new ExampleArmSubsystem.
     * 
     * @param motorIO The motor IO implementation
     * @param pidController PID controller for position control
     * @param feedforward Feedforward controller with kS, kG, kV, kA
     */
    public ExampleArmSubsystem(
            TalonFXMotorIO motorIO, 
            PIDController pidController,
            ArmFeedforward feedforward) {
        this.motorIO = motorIO;
        this.inputs = new TalonFXMotorInputsAutoLogged();
        this.pidController = pidController;
        this.feedforward = feedforward;
        
        pidController.setTolerance(POSITION_TOLERANCE_RAD);
    }
    
    /**
     * Factory method to create this subsystem with real hardware.
     * 
     * @param canId The CAN ID of the motor
     * @return A new ExampleArmSubsystem configured for real hardware
     */
    public static ExampleArmSubsystem createReal(int canId) {
        TalonFXMotorIOReal motorIOReal = new TalonFXMotorIOReal(canId);
        
        // Configure motor for arm control
        motorIOReal.getMotor().setPositionConversionFactor(2.0 * Math.PI / GEAR_RATIO); // Rotations to radians
        motorIOReal.getMotor().setVelocityConversionFactor(2.0 * Math.PI / GEAR_RATIO); // RPS to rad/s
        
        // PID constants - tune these for your arm
        PIDController pid = new PIDController(5.0, 0.0, 0.1);
        
        // Feedforward constants - tune these for your arm
        // kS: voltage to overcome friction, kG: voltage to hold against gravity,
        // kV: voltage per velocity, kA: voltage per acceleration
        ArmFeedforward ff = new ArmFeedforward(0.1, 0.5, 1.5, 0.05);
        
        return new ExampleArmSubsystem(motorIOReal, pid, ff);
    }
    
    /**
     * Factory method to create this subsystem for simulation/replay.
     * 
     * @return A new ExampleArmSubsystem configured for simulation
     */
    public static ExampleArmSubsystem createSim() {
        PIDController pid = new PIDController(5.0, 0.0, 0.1);
        ArmFeedforward ff = new ArmFeedforward(0.1, 0.5, 1.5, 0.05);
        return new ExampleArmSubsystem(new TalonFXMotorIO(), pid, ff);
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware and log them
        motorIO.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        
        // Log control state
        Logger.recordOutput("Arm/TargetPositionRad", targetPositionRadians);
        Logger.recordOutput("Arm/PositionErrorRad", getPositionError());
        Logger.recordOutput("Arm/AtSetpoint", atSetpoint());
    }
    
    /**
     * Command to move the arm to a target position.
     * 
     * @param positionRadians Target position in radians (0 = horizontal)
     * @return Command that moves the arm to the target
     */
    public Command moveToPosition(DoubleSupplier positionRadians) {
        return run(() -> {
            targetPositionRadians = positionRadians.getAsDouble();
            
            // Get current state
            double currentPosition = getPositionRadians();
            double currentVelocity = getVelocityRadPerSec();
            
            // Calculate PID output
            double pidOutput = pidController.calculate(currentPosition, targetPositionRadians);
            
            // Calculate feedforward
            // For an arm, feedforward depends on position (gravity) and velocity
            double ffOutput = feedforward.calculate(targetPositionRadians, currentVelocity);
            
            // Combine and apply
            double voltage = pidOutput + ffOutput;
            voltage = Math.max(-12.0, Math.min(12.0, voltage)); // Clamp to ±12V
            
            motorIO.setVoltage(voltage);
            
            Logger.recordOutput("Arm/PIDOutput", pidOutput);
            Logger.recordOutput("Arm/FFOutput", ffOutput);
            Logger.recordOutput("Arm/TotalVoltage", voltage);
        });
    }
    
    /**
     * Command to hold the current position.
     * 
     * @return Command that holds the current position
     */
    public Command holdPosition() {
        return runOnce(() -> targetPositionRadians = getPositionRadians())
            .andThen(moveToPosition(() -> targetPositionRadians));
    }
    
    /**
     * Command to manually control the arm with voltage.
     * 
     * @param voltageSupplier Voltage supplier (-12 to 12)
     * @return Command for manual control
     */
    public Command manualControl(DoubleSupplier voltageSupplier) {
        return run(() -> {
            double voltage = voltageSupplier.getAsDouble();
            voltage = Math.max(-12.0, Math.min(12.0, voltage));
            motorIO.setVoltage(voltage);
            
            // Update target to current position for smooth transition back to auto
            targetPositionRadians = getPositionRadians();
        });
    }
    
    /**
     * Stops the arm motor.
     */
    public void stop() {
        motorIO.stop();
    }
    
    /**
     * Command to stop the arm.
     * 
     * @return Command that stops the arm
     */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
    
    /**
     * Gets the current arm position in radians.
     * 
     * @return Position in radians (0 = horizontal)
     */
    public double getPositionRadians() {
        return inputs.positionRotations;
    }
    
    /**
     * Gets the current arm velocity in radians per second.
     * 
     * @return Velocity in rad/s
     */
    public double getVelocityRadPerSec() {
        return inputs.velocityRotationsPerSecond;
    }
    
    /**
     * Gets the position error.
     * 
     * @return Error in radians
     */
    public double getPositionError() {
        return targetPositionRadians - getPositionRadians();
    }
    
    /**
     * Checks if the arm is at the target setpoint.
     * 
     * @return True if within tolerance
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
    
    /**
     * Resets the arm position to a given value.
     * Useful for homing/zeroing the arm.
     * 
     * @param positionRadians The position to set
     */
    public void resetPosition(double positionRadians) {
        motorIO.setPosition(positionRadians);
        targetPositionRadians = positionRadians;
    }
}
