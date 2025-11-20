package frc.excalib.examples;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.excalib.control.imu.IMUIO;
import frc.excalib.control.imu.IMUInputsAutoLogged;
import frc.excalib.control.imu.PigeonIOReal;
import frc.excalib.control.motor.controllers.TalonFXMotorIO;
import frc.excalib.control.motor.controllers.TalonFXMotorIOReal;
import frc.excalib.control.motor.controllers.TalonFXMotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Example drive subsystem demonstrating AdvantageKit integration with motors and IMU.
 * 
 * This example shows:
 * - Multi-motor control with IO patterns
 * - IMU integration for heading tracking
 * - Complete replay support for all hardware
 * - Factory methods for real vs. simulation
 */
public class ExampleDriveSubsystem extends SubsystemBase {
    private final TalonFXMotorIO leftMotorIO;
    private final TalonFXMotorIO rightMotorIO;
    private final IMUIO gyroIO;
    
    private final TalonFXMotorInputsAutoLogged leftInputs;
    private final TalonFXMotorInputsAutoLogged rightInputs;
    private final IMUInputsAutoLogged gyroInputs;
    
    /**
     * Creates a new ExampleDriveSubsystem.
     * 
     * @param leftMotorIO Left motor IO implementation
     * @param rightMotorIO Right motor IO implementation
     * @param gyroIO Gyro IO implementation
     */
    public ExampleDriveSubsystem(TalonFXMotorIO leftMotorIO, TalonFXMotorIO rightMotorIO, IMUIO gyroIO) {
        this.leftMotorIO = leftMotorIO;
        this.rightMotorIO = rightMotorIO;
        this.gyroIO = gyroIO;
        
        this.leftInputs = new TalonFXMotorInputsAutoLogged();
        this.rightInputs = new TalonFXMotorInputsAutoLogged();
        this.gyroInputs = new IMUInputsAutoLogged();
    }
    
    /**
     * Factory method to create this subsystem with real hardware.
     * 
     * @param leftCanId Left motor CAN ID
     * @param rightCanId Right motor CAN ID
     * @param gyroCanId Gyro CAN ID
     * @return A new ExampleDriveSubsystem configured for real hardware
     */
    public static ExampleDriveSubsystem createReal(int leftCanId, int rightCanId, int gyroCanId) {
        return new ExampleDriveSubsystem(
            new TalonFXMotorIOReal(leftCanId),
            new TalonFXMotorIOReal(rightCanId),
            new PigeonIOReal(gyroCanId, new Rotation3d())
        );
    }
    
    /**
     * Factory method to create this subsystem for simulation/replay.
     * 
     * @return A new ExampleDriveSubsystem configured for simulation
     */
    public static ExampleDriveSubsystem createSim() {
        return new ExampleDriveSubsystem(
            new TalonFXMotorIO(),
            new TalonFXMotorIO(),
            new IMUIO()
        );
    }
    
    @Override
    public void periodic() {
        // Update all inputs from hardware and log them
        leftMotorIO.updateInputs(leftInputs);
        rightMotorIO.updateInputs(rightInputs);
        gyroIO.updateInputs(gyroInputs);
        
        Logger.processInputs("Drive/LeftMotor", leftInputs);
        Logger.processInputs("Drive/RightMotor", rightInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        
        // Log computed values
        Logger.recordOutput("Drive/AverageVelocityRPS", getAverageVelocity());
        Logger.recordOutput("Drive/HeadingDeg", getHeading().getDegrees());
    }
    
    /**
     * Arcade drive command with forward and rotation inputs.
     * 
     * @param forward Forward speed supplier (-1.0 to 1.0)
     * @param rotation Rotation speed supplier (-1.0 to 1.0)
     * @return Command that drives with arcade controls
     */
    public Command arcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
        return run(() -> {
            double fwd = forward.getAsDouble();
            double rot = rotation.getAsDouble();
            
            double leftPower = fwd + rot;
            double rightPower = fwd - rot;
            
            // Normalize if necessary
            double maxMagnitude = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMagnitude > 1.0) {
                leftPower /= maxMagnitude;
                rightPower /= maxMagnitude;
            }
            
            leftMotorIO.setPercentage(leftPower);
            rightMotorIO.setPercentage(rightPower);
            
            Logger.recordOutput("Drive/LeftPower", leftPower);
            Logger.recordOutput("Drive/RightPower", rightPower);
        });
    }
    
    /**
     * Tank drive command with left and right inputs.
     * 
     * @param left Left side speed supplier (-1.0 to 1.0)
     * @param right Right side speed supplier (-1.0 to 1.0)
     * @return Command that drives with tank controls
     */
    public Command tankDrive(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            leftMotorIO.setPercentage(left.getAsDouble());
            rightMotorIO.setPercentage(right.getAsDouble());
        });
    }
    
    /**
     * Stops both motors.
     */
    public void stop() {
        leftMotorIO.stop();
        rightMotorIO.stop();
    }
    
    /**
     * Command to stop the drive.
     * 
     * @return Command that stops both motors
     */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
    
    /**
     * Resets the gyro heading to zero.
     */
    public void resetHeading() {
        gyroIO.reset();
    }
    
    /**
     * Gets the current heading from the gyro.
     * 
     * @return Current heading as Rotation2d
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyroInputs.yawDegrees);
    }
    
    /**
     * Gets the average velocity of both sides.
     * 
     * @return Average velocity in rotations per second
     */
    public double getAverageVelocity() {
        return (leftInputs.velocityRotationsPerSecond + rightInputs.velocityRotationsPerSecond) / 2.0;
    }
    
    /**
     * Checks if the gyro is connected.
     * 
     * @return True if gyro is connected
     */
    public boolean isGyroConnected() {
        return gyroInputs.connected;
    }
}
