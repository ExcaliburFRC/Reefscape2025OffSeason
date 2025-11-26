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
    private final CommandPS5Controller driverController = new CommandPS5Controller(0);
    
    // Example Subsystems with AdvantageKit integration
    private final ExampleMotorSubsystem motorSubsystem;
    private final ExampleFlyWheelSubsystem flywheelSubsystem;
    private final ExampleDriveSubsystem driveSubsystem;
    private final ExampleArmSubsystem armSubsystem;
    private final ExampleTurretSubsystem turretSubsystem;
    private final ExampleSwerveSubsystem swerveSubsystem;
    
    public RobotContainerWithExamples() {
        // Initialize subsystems based on robot mode
        if (RobotBase.isReal()) {
            // Real robot - use actual hardware
            motorSubsystem = ExampleMotorSubsystem.createReal(1);
            flywheelSubsystem = ExampleFlyWheelSubsystem.createReal(2, createFlywheelGains());
            driveSubsystem = ExampleDriveSubsystem.createReal(3, 4, 5);
            armSubsystem = ExampleArmSubsystem.createReal(6);
            turretSubsystem = ExampleTurretSubsystem.createReal(7);
            // Note: Swerve requires actual swerve modules - see comments below for setup
            swerveSubsystem = ExampleSwerveSubsystem.createSim();
        } else {
            // Simulation or replay - use simulated hardware
            motorSubsystem = ExampleMotorSubsystem.createSim();
            flywheelSubsystem = ExampleFlyWheelSubsystem.createSim(createFlywheelGains());
            driveSubsystem = ExampleDriveSubsystem.createSim();
            armSubsystem = ExampleArmSubsystem.createSim();
            turretSubsystem = ExampleTurretSubsystem.createSim();
            swerveSubsystem = ExampleSwerveSubsystem.createSim();
        }
        
        configureBindings();
        configureDefaultCommands();
    }

    // Simple helper to log a message and then run a Runnable
    private void logAndRun(String msg, Runnable r) {
        System.out.println(msg);
        r.run();
    }

    // Public helper you can call from anywhere to indicate a function was invoked
    public void notifyFunctionCalled(String name) {
        System.out.println("Function called: " + name);
    }

    /**
     * Configure button bindings for the simulated controller.
     */
    private void configureBindings() {
        // ========== Drive Subsystem Controls ==========
        // Left stick = drive forward/strafe
        // Right stick X = rotate
        // These are set up in configureDefaultCommands()
        
        // A Button = b gyro heading
        driverController.triangle().onTrue(
            Commands.runOnce(() -> logAndRun("Triagnle pressed: ResetHeading", () -> driveSubsystem.resetHeading()), driveSubsystem)
                .withName("ResetHeading")
        );
        
        // B Button = Stop drive
        driverController.square().onTrue(
            Commands.runOnce(() -> logAndRun("square pressed: Stop drive", () -> driveSubsystem.stopCommand().schedule()))
        );

        // ========== Arm Subsystem Controls ==========
        // Y Button = Move arm to 45 degrees
        driverController.cross().onTrue(
            Commands.runOnce(() -> logAndRun("cross pressed: ArmTo45Deg", () -> armSubsystem.moveToPosition(() -> Math.PI / 4).schedule()))
        );
        
        // X Button = Move arm to horizontal (0 degrees)
        driverController.circle().onTrue(
            Commands.runOnce(() -> logAndRun("circle pressed: ArmToHorizontal", () -> armSubsystem.moveToPosition(() -> 0.0).schedule()))
        );
        
        // Left Bumper = Move arm to 90 degrees (vertical)
        driverController.L1().onTrue(
            Commands.runOnce(() -> logAndRun("Left Bumper pressed: ArmToVertical", () -> armSubsystem.moveToPosition(() -> Math.PI / 2).schedule()))
        );
        
        // Right Bumper = Arm manual control (right trigger controls voltage)
        driverController.R1().onTrue(
            Commands.runOnce(() -> logAndRun("Right Bumper pressed: ArmManual", () -> armSubsystem.manualControl(() -> driverController.getR2Axis() * 3.0).schedule()))
        );
        
        // ========== Turret Subsystem Controls ==========
        // D-Pad Up = Turret to 0 degrees
        driverController.povUp().onTrue(
            Commands.runOnce(() -> logAndRun("POV Up pressed: TurretTo0", () -> turretSubsystem.rotateToAngleAndHold(new Rotation2d()).withTimeout(2.0).schedule()))
        );
        
        // D-Pad Right = Turret to 90 degrees
        driverController.povRight().onTrue(
            Commands.runOnce(() -> logAndRun("POV Right pressed: TurretTo90", () -> turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(90)).withTimeout(2.0).schedule()))
        );
        
        // D-Pad Down = Turret to 180 degrees
        driverController.povDown().onTrue(
            Commands.runOnce(() -> logAndRun("POV Down pressed: TurretTo180", () -> turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(180)).withTimeout(2.0).schedule()))
        );
        
        // D-Pad Left = Turret to -90 degrees
        driverController.povLeft().onTrue(
            Commands.runOnce(() -> logAndRun("POV Left pressed: TurretToNeg90", () -> turretSubsystem.rotateToAngleAndHold(Rotation2d.fromDegrees(-90)).withTimeout(2.0).schedule()))
        );
        
        // ========== Flywheel Subsystem Controls ==========
        // Start Button = Run flywheel at 50 RPS
        driverController.options().onTrue(
            Commands.runOnce(() -> logAndRun("Start pressed: FlywheelRun", () -> flywheelSubsystem.runVelocity(() -> 50.0).schedule()))
        );
        
        // Back Button = Stop flywheel
        driverController.create().onTrue(
            Commands.runOnce(() -> logAndRun("Back pressed: FlywheelStop", () -> flywheelSubsystem.stopCommand().schedule()))
        );

        // ========== Motor Subsystem Controls ==========
        // Left Trigger = Run simple motor forward
//         Print when trigger crosses threshold, keep original whileTrue for continuous control
//        driverController.getL2Axis().onTrue(
//            Commands.runOnce(() -> System.out.println("Left Trigger pressed: starting simple motor"))
//        );
//        driverController.getR2Axis().whileTrue(
//            Commands.run(() -> motorSubsystem.setVoltage(6.0), motorSubsystem)
//                .withName("MotorForward")
//        );
        
        // ========== Swerve Subsystem Controls ==========
        // Touchpad = Reset swerve pose
        driverController.touchpad().onTrue(
            Commands.runOnce(() -> logAndRun("Touchpad pressed: ResetSwervePose", 
                () -> swerveSubsystem.resetPose(new edu.wpi.first.math.geometry.Pose2d())), 
                swerveSubsystem)
                .withName("ResetSwervePose")
        );
        
        // PS Button = Stop swerve
        driverController.PS().onTrue(
            Commands.runOnce(() -> logAndRun("PS pressed: StopSwerve", 
                () -> swerveSubsystem.stopCommand().schedule()))
        );
    }

    /**
     * yehuda.2009
     * Configure default commands that run continuously.
     */
    private void configureDefaultCommands() {
        // Swerve subsystem default command - field-centric drive
        // Uses left stick for translation and right stick for rotation
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.driveFieldCentric(
                () -> -applyDeadband(driverController.getLeftY()) * swerveSubsystem.getMaxVelocity() * 0.8,  // Forward/backward (80% speed)
                () -> -applyDeadband(driverController.getLeftX()) * swerveSubsystem.getMaxVelocity() * 0.8,  // Strafe (80% speed)
                () -> -applyDeadband(driverController.getRightX()) * swerveSubsystem.getMaxAngularVelocity() * 0.6  // Rotation (60% speed)
            ).withName("DefaultSwerveDrive")
        );
        
        // Drive subsystem default command - arcade drive (tank drive example)
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
            Commands.runOnce(() -> { logAndRun("Autonomous starting", () -> {}); }),
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
    public boolean isSwerveGyroConnected() {
        return swerveSubsystem.isGyroConnected();
    }
    
    @Log.NT
    public double getSwerveHeadingDegrees() {
        return swerveSubsystem.getHeading().getDegrees();
    }
    
    @Log.NT
    public double getSwervePoseX() {
        return swerveSubsystem.getPose().getX();
    }
    
    @Log.NT
    public double getSwervePoseY() {
        return swerveSubsystem.getPose().getY();
    }
    
    @Log.NT
    public boolean isFlywheelAtSpeed() {
        return flywheelSubsystem.isAtSetpoint();
    }
}
