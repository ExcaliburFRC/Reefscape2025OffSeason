// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.excalib.control.gains.Gains;
import frc.excalib.examples.*;
import monologue.Logged;

import static monologue.Annotations.Log;

/**
 * RobotContainer with complete AdvantageKit example subsystems.
 * This demonstrates how to integrate all the example subsystems with proper replay support.
 * 
 * Connect a simulated controller (port 0) to test all functionality.
 */
public class RobotContainerWithExamples implements Logged {

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    
    // Example Subsystems with AdvantageKit integration
    private final ExampleMotorSubsystem motorSubsystem;
    private final ExampleFlyWheelSubsystem flywheelSubsystem;
    private final ExampleDriveSubsystem driveSubsystem;
    private final ExampleArmSubsystem armSubsystem;
    private final ExampleTurretSubsystem turretSubsystem;
    
    public RobotContainerWithExamples() {
        // Initialize subsystems based on robot mode
        if (RobotBase.isReal()) {
            // Real robot - use actual hardware
            motorSubsystem = ExampleMotorSubsystem.createReal(1);
            flywheelSubsystem = ExampleFlyWheelSubsystem.createReal(2, createFlywheelGains());
            driveSubsystem = ExampleDriveSubsystem.createReal(3, 4, 5);
            armSubsystem = ExampleArmSubsystem.createReal(6);
            turretSubsystem = ExampleTurretSubsystem.createReal(7);
        } else {
            // Simulation or replay - use simulated hardware
            motorSubsystem = ExampleMotorSubsystem.createSim();
            flywheelSubsystem = ExampleFlyWheelSubsystem.createSim(createFlywheelGains());
            driveSubsystem = ExampleDriveSubsystem.createSim();
            armSubsystem = ExampleArmSubsystem.createSim();
            turretSubsystem = ExampleTurretSubsystem.createSim();
        }
        
        configureBindings();
        configureDefaultCommands();
    }
    
    /**
     * Configure button bindings for the simulated controller.
     */
    private void configureBindings() {
        // ========== Drive Subsystem Controls ==========
        // Left stick = drive forward/strafe
        // Right stick X = rotate
        // These are set up in configureDefaultCommands()
        
        // A Button = Reset gyro heading
        driverController.a().onTrue(
            Commands.runOnce(() -> driveSubsystem.resetHeading(), driveSubsystem)
                .withName("ResetHeading")
        );
        
        // B Button = Stop drive
        driverController.b().onTrue(driveSubsystem.stopCommand());
        
        // ========== Arm Subsystem Controls ==========
        // Y Button = Move arm to 45 degrees
        driverController.y().whileTrue(
            armSubsystem.moveToPosition(() -> Math.PI / 4)
                .withName("ArmTo45Deg")
        );
        
        // X Button = Move arm to horizontal (0 degrees)
        driverController.x().whileTrue(
            armSubsystem.moveToPosition(() -> 0.0)
                .withName("ArmToHorizontal")
        );
        
        // Left Bumper = Move arm to 90 degrees (vertical)
        driverController.leftBumper().whileTrue(
            armSubsystem.moveToPosition(() -> Math.PI / 2)
                .withName("ArmToVertical")
        );
        
        // Right Bumper = Arm manual control (right trigger controls voltage)
        driverController.rightBumper().whileTrue(
            armSubsystem.manualControl(() -> driverController.getRightTriggerAxis() * 3.0)
                .withName("ArmManual")
        );
        
        // ========== Turret Subsystem Controls ==========
        // D-Pad Up = Turret to 0 degrees
        driverController.povUp().onTrue(
            turretSubsystem.rotateToAngleAndHold(new Rotation2d())
                .withTimeout(2.0)
                .withName("TurretTo0")
        );
        
        // D-Pad Right = Turret to 90 degrees
        driverController.povRight().onTrue(
            turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(90))
                .withTimeout(2.0)
                .withName("TurretTo90")
        );
        
        // D-Pad Down = Turret to 180 degrees
        driverController.povDown().onTrue(
            turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(180))
                .withTimeout(2.0)
                .withName("TurretTo180")
        );
        
        // D-Pad Left = Turret to -90 degrees
        driverController.povLeft().onTrue(
            turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(-90))
                .withTimeout(2.0)
                .withName("TurretToNeg90")
        );
        
        // ========== Flywheel Subsystem Controls ==========
        // Start Button = Run flywheel at 50 RPS
        driverController.start().whileTrue(
            flywheelSubsystem.runVelocity(() -> 50.0)
                .withName("FlywheelRun")
        );
        
        // Back Button = Stop flywheel
        driverController.back().onTrue(flywheelSubsystem.stopCommand());
        
        // ========== Motor Subsystem Controls ==========
        // Left Trigger = Run simple motor forward
        driverController.leftTrigger(0.1).whileTrue(
            Commands.run(() -> motorSubsystem.setVoltage(6.0), motorSubsystem)
                .withName("MotorForward")
        );
    }
    
    /**
     * Configure default commands that run continuously.
     */
    private void configureDefaultCommands() {
        // Drive subsystem default command - arcade drive
        driveSubsystem.setDefaultCommand(
            driveSubsystem.arcadeDrive(
                () -> -applyDeadband(driverController.getLeftY()) * 0.5, // Forward/backward (50% speed)
                () -> -applyDeadband(driverController.getRightX()) * 0.3  // Rotation (30% speed)
            ).withName("DefaultDrive")
        );
        
        // Arm subsystem default command - hold current position
        armSubsystem.setDefaultCommand(
            armSubsystem.holdPosition()
                .withName("ArmHold")
        );
    }
    
    /**
     * Apply deadband to controller inputs.
     */
    private double applyDeadband(double value) {
        return Math.abs(value) < 0.09 ? 0.0 : value;
    }
    
    /**
     * Create gains for the flywheel subsystem.
     */
    private Gains createFlywheelGains() {
        return new Gains(
            0.5,  // kp
            0.0,  // ki
            0.0,  // kd
            0.1,  // ks
            0.12, // kv
            0.01, // ka
            0.0   // kg
        );
    }
    
    /**
     * Get the autonomous command.
     * This is a simple demonstration that moves the arm and spins the turret.
     */
    public Command getAutonomousCommand() {
        return Commands.sequence(
            Commands.print("=== Starting Autonomous ==="),
            
            // Move arm to 45 degrees
            Commands.parallel(
                armSubsystem.moveToPosition(() -> Math.PI / 4),
                Commands.waitSeconds(2.0)
            ),
            
            // Spin turret 360 degrees
            turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(180))
                .withTimeout(2.0),
            turretSubsystem.rotateToAngleAndHold(new Rotation2d())
                .withTimeout(2.0),
            
            // Run flywheel
            Commands.parallel(
                flywheelSubsystem.runVelocity(() -> 30.0),
                Commands.waitSeconds(3.0)
            ),
            
            // Stop everything
            Commands.parallel(
                flywheelSubsystem.stopCommand(),
                turretSubsystem.stopCommand(),
                armSubsystem.stopCommand()
            ),
            
            Commands.print("=== Autonomous Complete ===")
        ).withName("ExampleAuto");
    }
    
    // ========== Logging Methods ==========
    
    @Log.NT
    public boolean isGyroConnected() {
        return driveSubsystem.isGyroConnected();
    }
    
    @Log.NT
    public double getHeadingDegrees() {
        return driveSubsystem.getHeading().getDegrees();
    }
    
    @Log.NT
    public double getArmPositionDegrees() {
        return Math.toDegrees(armSubsystem.getPositionRadians());
    }
    
    @Log.NT
    public boolean isArmAtSetpoint() {
        return armSubsystem.atSetpoint();
    }
    
    @Log.NT
    public double getTurretAngleDegrees() {
        return turretSubsystem.getAngle().getDegrees();
    }
    
    @Log.NT
    public boolean isTurretAtSetpoint() {
        return turretSubsystem.atSetpoint();
    }
    
    @Log.NT
    public double getFlywheelVelocityRPS() {
        return flywheelSubsystem.getVelocityRPS();
    }
    
    @Log.NT
    public boolean isFlywheelAtSpeed() {
        return flywheelSubsystem.isAtSetpoint();
    }
}
