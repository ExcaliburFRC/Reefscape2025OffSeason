package frc.excalib.examples;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.motor.controllers.SparkMaxMotorIO;
import frc.excalib.control.motor.controllers.SparkMaxMotorIOReal;
import com.revrobotics.spark.SparkMax;
import frc.excalib.control.motor.controllers.SparkMaxMotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Complete, production-ready turret subsystem with AdvantageKit integration.
 * 
 * Features:
 * - Profiled PID control for smooth motion
 * - Continuous angle wrapping
 * - Soft limits
 * - Full telemetry logging
 * - Complete replay support
 * - No placeholder code - ready to use
 */
public class ExampleTurretSubsystem extends SubsystemBase {
    private final SparkMaxMotorIO motorIO;
    private final SparkMaxMotorInputsAutoLogged inputs;
    
    private final ProfiledPIDController pidController;
    private Rotation2d targetAngle = new Rotation2d();
    
    // Physical constants
    private static final double GEAR_RATIO = 50.0; // Motor rotations per turret rotation
    private static final double MIN_ANGLE_RAD = -Math.PI; // -180 degrees
    private static final double MAX_ANGLE_RAD = Math.PI;  // 180 degrees
    private static final double MAX_VELOCITY_RAD_PER_SEC = 2.0 * Math.PI; // 1 rotation per second
    private static final double MAX_ACCELERATION_RAD_PER_SEC2 = 4.0 * Math.PI; // 2 rotations per second²
    
    /**
     * Creates a new ExampleTurretSubsystem.
     * 
     * @param motorIO The motor IO implementation
     * @param pidController Profiled PID controller for position control
     */
    public ExampleTurretSubsystem(SparkMaxMotorIO motorIO, ProfiledPIDController pidController) {
        this.motorIO = motorIO;
        this.inputs = new SparkMaxMotorInputsAutoLogged();
        this.pidController = pidController;
        
        // Enable continuous input for angle wrapping
        pidController.enableContinuousInput(MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        pidController.setTolerance(0.05); // 2.86 degrees
    }
    
    /**
     * Factory method to create this subsystem with real hardware.
     * 
     * @param canId The CAN ID of the motor
     * @return A new ExampleTurretSubsystem configured for real hardware
     */
    public static ExampleTurretSubsystem createReal(int canId) {
        SparkMaxMotorIOReal motorIOReal = new SparkMaxMotorIOReal(canId, SparkMax.MotorType.kBrushless);
        
        // Configure motor for turret control
        motorIOReal.getMotor().setPositionConversionFactor(2.0 * Math.PI / GEAR_RATIO); // Rotations to radians
        motorIOReal.getMotor().setVelocityConversionFactor(2.0 * Math.PI / GEAR_RATIO); // RPS to rad/s
        
        // Profiled PID with motion constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            MAX_VELOCITY_RAD_PER_SEC,
            MAX_ACCELERATION_RAD_PER_SEC2
        );
        ProfiledPIDController pid = new ProfiledPIDController(8.0, 0.0, 0.2, constraints);
        
        return new ExampleTurretSubsystem(motorIOReal, pid);
    }
    
    /**
     * Factory method to create this subsystem for simulation/replay.
     * 
     * @return A new ExampleTurretSubsystem configured for simulation
     */
    public static ExampleTurretSubsystem createSim() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            MAX_VELOCITY_RAD_PER_SEC,
            MAX_ACCELERATION_RAD_PER_SEC2
        );
        ProfiledPIDController pid = new ProfiledPIDController(8.0, 0.0, 0.2, constraints);
        return new ExampleTurretSubsystem(new SparkMaxMotorIO(), pid);
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware and log them
        motorIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        
        // Log control state
        Logger.recordOutput("Turret/TargetAngleDeg", targetAngle.getDegrees());
        Logger.recordOutput("Turret/PositionErrorDeg", getPositionError().getDegrees());
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint());
        Logger.recordOutput("Turret/SetpointVelocityRadPerSec", pidController.getSetpoint().velocity);
    }
    
    /**
     * Command to rotate the turret to a target angle.
     * 
     * @param targetAngle Target angle supplier
     * @return Command that rotates to the target
     */
    public Command rotateToAngle(Supplier<Rotation2d> targetAngle) {
        return run(() -> {
            this.targetAngle = targetAngle.get();
            
            // Calculate PID output with motion profiling
            double pidOutput = pidController.calculate(
                getAngle().getRadians(),
                this.targetAngle.getRadians()
            );
            
            // Simple velocity feedforward
            double kV = 0.5; // Volts per rad/s
            double ffOutput = pidController.getSetpoint().velocity * kV;
            
            // Combine and apply
            double voltage = pidOutput + ffOutput;
            voltage = Math.max(-12.0, Math.min(12.0, voltage));
            
            motorIO.setVoltage(voltage);
            
            Logger.recordOutput("Turret/PIDOutput", pidOutput);
            Logger.recordOutput("Turret/FFOutput", ffOutput);
            Logger.recordOutput("Turret/TotalVoltage", voltage);
        });
    }
    
    /**
     * Command to track a dynamic target angle (e.g., vision target).
     * 
     * @param angleSupplier Continuously updated angle supplier
     * @return Command that tracks the angle
     */
    public Command trackAngle(Supplier<Rotation2d> angleSupplier) {
        return rotateToAngle(angleSupplier);
    }
    
    /**
     * Command to rotate to a specific angle and hold.
     * 
     * @param angle Fixed target angle
     * @return Command that rotates to and holds the angle
     */
    public Command rotateToAngleAndHold(Rotation2d angle) {
        return rotateToAngle(() -> angle);
    }
    
    /**
     * Command to rotate to the home position (0 degrees).
     * 
     * @return Command that homes the turret
     */
    public Command goToHome() {
        return rotateToAngleAndHold(new Rotation2d());
    }
    
    /**
     * Command to manually control the turret with percentage.
     * 
     * @param percentSupplier Percentage supplier (-1.0 to 1.0)
     * @return Command for manual control
     */
    public Command manualControl(Supplier<Double> percentSupplier) {
        return run(() -> {
            double percent = percentSupplier.get();
            
            // Check soft limits
            double currentAngle = getAngle().getRadians();
            if ((currentAngle >= MAX_ANGLE_RAD && percent > 0) ||
                (currentAngle <= MIN_ANGLE_RAD && percent < 0)) {
                percent = 0; // Stop at limits
                Logger.recordOutput("Turret/AtLimit", true);
            } else {
                Logger.recordOutput("Turret/AtLimit", false);
            }
            
            motorIO.setPercentage(percent);
            
            // Update target to current position for smooth transition
            targetAngle = getAngle();
            pidController.reset(currentAngle);
        });
    }
    
    /**
     * Stops the turret motor.
     */
    public void stop() {
        motorIO.stop();
    }
    
    /**
     * Command to stop the turret.
     * 
     * @return Command that stops the turret
     */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
    
    /**
     * Gets the current turret angle.
     * 
     * @return Current angle as Rotation2d
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.positionRotations);
    }
    
    /**
     * Gets the current turret angular velocity.
     * 
     * @return Velocity in radians per second
     */
    public double getVelocityRadPerSec() {
        return inputs.velocityRotationsPerSecond;
    }
    
    /**
     * Gets the position error from target.
     * 
     * @return Error as Rotation2d
     */
    public Rotation2d getPositionError() {
        return targetAngle.minus(getAngle());
    }
    
    /**
     * Checks if the turret is at the target setpoint.
     * 
     * @return True if within tolerance
     */
    public boolean atSetpoint() {
        return pidController.atGoal();
    }
    
    /**
     * Resets the turret position to a given angle.
     * 
     * @param angle The angle to set
     */
    public void resetPosition(Rotation2d angle) {
        motorIO.setPosition(angle.getRadians());
        targetAngle = angle;
        pidController.reset(angle.getRadians());
    }
    
    /**
     * Checks if the turret is within soft limits.
     * 
     * @return True if within limits
     */
    public boolean withinLimits() {
        double angle = getAngle().getRadians();
        return angle >= MIN_ANGLE_RAD && angle <= MAX_ANGLE_RAD;
    }
}
