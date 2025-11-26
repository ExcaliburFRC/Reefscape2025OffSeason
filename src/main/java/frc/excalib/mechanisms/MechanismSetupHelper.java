package frc.excalib.mechanisms;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.Motor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import frc.excalib.mechanisms.turret.Turret;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.limits.SoftLimit;

import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * Helper class for setting up excalib mechanisms with common configurations.
 * Provides factory methods with sensible defaults to make mechanism setup easier.
 * 
 * Usage:
 * <pre>
 * // Create a flywheel with default gains
 * FlyWheel flywheel = MechanismSetupHelper.createFlyWheel(motor);
 * 
 * // Create a turret with custom gains
 * Turret turret = MechanismSetupHelper.createTurret(motor, angleSupplier, customGains);
 * </pre>
 */
public class MechanismSetupHelper {
    
    // ========== Default Gain Values ==========
    
    /**
     * Default gains for flywheel velocity control.
     * These values work well for typical FRC flywheels.
     */
    public static final Gains DEFAULT_FLYWHEEL_GAINS = new Gains(
        0.5,  // kp - proportional gain
        0.0,  // ki - integral gain
        0.0,  // kd - derivative gain
        0.1,  // ks - static friction compensation
        0.12, // kv - velocity feedforward
        0.01, // ka - acceleration feedforward
        0.0   // kg - gravity compensation (not needed for flywheel)
    );
    
    /**
     * Default gains for turret position control.
     * These values work well for typical FRC turrets.
     */
    public static final Gains DEFAULT_TURRET_GAINS = new Gains(
        8.0,  // kp - proportional gain
        0.0,  // ki - integral gain
        0.2,  // kd - derivative gain
        0.1,  // ks - static friction compensation
        0.5,  // kv - velocity feedforward
        0.05, // ka - acceleration feedforward
        0.0   // kg - gravity compensation (not needed for turret)
    );
    
    /**
     * Default gains for arm position control.
     * These values work well for typical FRC arms.
     */
    public static final Gains DEFAULT_ARM_GAINS = new Gains(
        5.0,  // kp - proportional gain
        0.0,  // ki - integral gain
        0.1,  // kd - derivative gain
        0.1,  // ks - static friction compensation
        1.5,  // kv - velocity feedforward
        0.05, // ka - acceleration feedforward
        0.5   // kg - gravity compensation
    );
    
    /**
     * Default gains for linear extension (elevator) control.
     * These values work well for typical FRC elevators.
     */
    public static final Gains DEFAULT_EXTENSION_GAINS = new Gains(
        4.0,  // kp - proportional gain
        0.0,  // ki - integral gain
        0.1,  // kd - derivative gain
        0.2,  // ks - static friction compensation
        1.0,  // kv - velocity feedforward
        0.05, // ka - acceleration feedforward
        0.3   // kg - gravity compensation
    );
    
    // ========== FlyWheel Factory Methods ==========
    
    /**
     * Creates a FlyWheel mechanism with default gains.
     * 
     * @param motor The motor controlling the flywheel
     * @return Configured FlyWheel mechanism
     */
    public static FlyWheel createFlyWheel(Motor motor) {
        return createFlyWheel(motor, DEFAULT_FLYWHEEL_GAINS);
    }
    
    /**
     * Creates a FlyWheel mechanism with custom gains.
     * 
     * @param motor The motor controlling the flywheel
     * @param gains Custom PID and feedforward gains
     * @return Configured FlyWheel mechanism
     */
    public static FlyWheel createFlyWheel(Motor motor, Gains gains) {
        return new FlyWheel(
            motor,
            100.0,  // maxAcceleration (rad/s²) - adjust based on your flywheel
            1000.0, // maxJerk (rad/s³) - adjust based on your flywheel
            gains
        );
    }
    
    // ========== Turret Factory Methods ==========
    
    /**
     * Creates a Turret mechanism with default gains and no limits.
     * 
     * @param motor The motor controlling the turret
     * @param positionSupplier Supplier for current turret position (radians)
     * @return Configured Turret mechanism
     */
    public static Turret createTurret(Motor motor, DoubleSupplier positionSupplier) {
        return createTurret(motor, positionSupplier, DEFAULT_TURRET_GAINS);
    }
    
    /**
     * Creates a Turret mechanism with custom gains and no limits.
     * 
     * @param motor The motor controlling the turret
     * @param positionSupplier Supplier for current turret position (radians)
     * @param gains Custom PID and feedforward gains
     * @return Configured Turret mechanism
     */
    public static Turret createTurret(Motor motor, DoubleSupplier positionSupplier, Gains gains) {
        // No soft limits - full 360° rotation
        ContinuousSoftLimit noLimits = new ContinuousSoftLimit(
            () -> Double.NEGATIVE_INFINITY,
            () -> Double.POSITIVE_INFINITY
        );
        
        return new Turret(
            motor,
            noLimits,
            gains,
            0.05, // PID tolerance (radians) - ~2.86 degrees
            positionSupplier
        );
    }
    
    /**
     * Creates a Turret mechanism with custom gains and soft limits.
     * 
     * @param motor The motor controlling the turret
     * @param positionSupplier Supplier for current turret position (radians)
     * @param gains Custom PID and feedforward gains
     * @param minAngleRad Minimum angle limit (radians)
     * @param maxAngleRad Maximum angle limit (radians)
     * @return Configured Turret mechanism
     */
    public static Turret createTurretWithLimits(
            Motor motor,
            DoubleSupplier positionSupplier,
            Gains gains,
            double minAngleRad,
            double maxAngleRad) {
        
        ContinuousSoftLimit limits = new ContinuousSoftLimit(
            () -> minAngleRad,
            () -> maxAngleRad
        );
        
        return new Turret(
            motor,
            limits,
            gains,
            0.05, // PID tolerance (radians)
            positionSupplier
        );
    }
    
    // ========== Arm Factory Methods ==========
    
    /**
     * Creates an Arm mechanism with default gains.
     * 
     * @param motor The motor controlling the arm
     * @param angleSupplier Supplier for current arm angle (radians)
     * @param armMassKg Mass of the arm (kilograms)
     * @return Configured Arm mechanism
     */
    public static Arm createArm(Motor motor, DoubleSupplier angleSupplier, double armMassKg) {
        return createArm(motor, angleSupplier, armMassKg, DEFAULT_ARM_GAINS);
    }
    
    /**
     * Creates an Arm mechanism with custom gains.
     * 
     * @param motor The motor controlling the arm
     * @param angleSupplier Supplier for current arm angle (radians)
     * @param armMassKg Mass of the arm (kilograms)
     * @param gains Custom PID and feedforward gains
     * @return Configured Arm mechanism
     */
    public static Arm createArm(
            Motor motor,
            DoubleSupplier angleSupplier,
            double armMassKg,
            Gains gains) {
        
        // No velocity limits by default
        SoftLimit noLimits = new SoftLimit(
            () -> Double.NEGATIVE_INFINITY,
            () -> Double.POSITIVE_INFINITY
        );
        
        Mass mass = new Mass(()-> Math.cos(angleSupplier.getAsDouble()), ()-> Math.sin(angleSupplier.getAsDouble()), armMassKg);
        
        return new Arm(
            motor,
            angleSupplier,
            noLimits,
            gains,
            mass
        );
    }
    
    // ========== Linear Extension (Elevator) Factory Methods ==========
    
    /**
     * Creates a LinearExtension mechanism with default gains.
     * 
     * @param motor The motor controlling the extension
     * @param positionSupplier Supplier for current position (meters)
     * @param angleSupplier Supplier for current angle (radians, for gravity compensation)
     * @return Configured LinearExtension mechanism
     */
    public static LinearExtension createLinearExtension(
            Motor motor,
            DoubleSupplier positionSupplier,
            DoubleSupplier angleSupplier) {
        return createLinearExtension(motor, positionSupplier, angleSupplier, DEFAULT_EXTENSION_GAINS);
    }
    
    /**
     * Creates a LinearExtension mechanism with custom gains.
     * 
     * @param motor The motor controlling the extension
     * @param positionSupplier Supplier for current position (meters)
     * @param angleSupplier Supplier for current angle (radians, for gravity compensation)
     * @param gains Custom PID and feedforward gains
     * @return Configured LinearExtension mechanism
     */
    public static LinearExtension createLinearExtension(
            Motor motor,
            DoubleSupplier positionSupplier,
            DoubleSupplier angleSupplier,
            Gains gains) {
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            2.0,  // maxVelocity (m/s) - adjust based on your extension
            4.0   // maxAcceleration (m/s²) - adjust based on your extension
        );
        
        return new LinearExtension(
            motor,
            positionSupplier,
            angleSupplier,
            gains,
            constraints,
            0.02 // tolerance (meters) - 2cm
        );
    }
    
    // ========== Utility Methods ==========
    
    /**
     * Creates a PIDController with values from Gains.
     * Useful for manual control setup.
     * 
     * @param gains Gains containing PID values
     * @return Configured PIDController
     */
    public static PIDController createPIDController(Gains gains) {
        return new PIDController(gains.kp, gains.ki, gains.kd);
    }
    
    /**
     * Creates an ArmFeedforward with values from Gains.
     * Useful for arm control.
     * 
     * @param gains Gains containing feedforward values
     * @return Configured ArmFeedforward
     */
    public static ArmFeedforward createArmFeedforward(Gains gains) {
        return new ArmFeedforward(gains.ks, gains.kg, gains.kv, gains.ka);
    }
    
    /**
     * Creates a ProfiledPIDController with values from Gains.
     * Useful for motion profiled control.
     * 
     * @param gains Gains containing PID values
     * @param maxVelocity Maximum velocity for motion profile
     * @param maxAcceleration Maximum acceleration for motion profile
     * @return Configured ProfiledPIDController
     */
    public static ProfiledPIDController createProfiledPIDController(
            Gains gains,
            double maxVelocity,
            double maxAcceleration) {
        
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            maxVelocity,
            maxAcceleration
        );
        
        return new ProfiledPIDController(gains.kp, gains.ki, gains.kd, constraints);
    }
}
