// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot class that uses RobotContainerWithExamples to demonstrate
 * AdvantageKit integration with all example subsystems.
 * 
 * To use this instead of the default Robot.java:
 * 1. In Main.java, change RobotBase.startRobot(Robot::new) to RobotBase.startRobot(RobotWithExamples::new)
 * 2. Or rename this file to Robot.java and rename the original Robot.java to RobotOriginal.java
 * 
 * This setup includes:
 * - Complete AdvantageKit logging for all hardware
 * - Replay support for all subsystems
 * - Simulated controller bindings (Xbox controller on port 0)
 * - All 5 example subsystems integrated and functional
 */
public class RobotWithExamples extends LoggedRobot implements Logged {
    private Command autonomousCommand;
    private final RobotContainerWithExamples robotContainer;
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    
    // Set this to false to run in simulation mode with logging
    // Set to true to replay logs from AdvantageScope
    public static boolean isReplay = false;
    
    public RobotWithExamples() {
        // Configure AdvantageKit logging
        Logger.recordMetadata("ProjectName", "Reefscape2025-Examples");
        Logger.recordMetadata("GitSHA", "AdvantageKit-Integration");
        
        if (RobotBase.isReal()) {
            // Real robot - log to USB and publish to NetworkTables
            Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else if (isReplay) {
            // Replay mode - run as fast as possible
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            Logger.recordOutput("ReplayMode", true);
        } else {
            // Simulation mode - log to temp directory and publish to NetworkTables
            Logger.addDataReceiver(new WPILOGWriter(""));
            Logger.addDataReceiver(new NT4Publisher());
            Logger.recordOutput("SimulationMode", true);
        }
        
        // Start logging
        Logger.start();
        
        // Initialize robot container with all example subsystems
        robotContainer = new RobotContainerWithExamples();
        
        Logger.recordOutput("RobotInitialized", true);
    }
    
    @Override
    public void robotInit() {
        Logger.recordOutput("RobotInit/Complete", true);
    }
    
    @Override
    public void robotPeriodic() {
        // Run motor status updates with high priority
        Threads.setCurrentThreadPriority(true, 99);
        TalonFXMotor.refreshAll();
        Monologue.updateAll();
        Threads.setCurrentThreadPriority(false, 10);
        
        // Run command scheduler
        commandScheduler.run();
        
        // Log scheduler info
        Logger.recordOutput("CommandScheduler/CommandCount", 
            commandScheduler.getScheduledCommands().size());
    }
    
    @Override
    public void disabledInit() {
        Logger.recordOutput("RobotMode", "Disabled");
    }
    
    @Override
    public void disabledPeriodic() {
    }
    
    @Override
    public void disabledExit() {
    }
    
    @Override
    public void autonomousInit() {
        Logger.recordOutput("RobotMode", "Autonomous");
        
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
            Logger.recordOutput("Auto/CommandScheduled", true);
        } else {
            Logger.recordOutput("Auto/CommandScheduled", false);
        }
    }
    
    @Override
    public void autonomousPeriodic() {
    }
    
    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
    
    @Override
    public void teleopInit() {
        Logger.recordOutput("RobotMode", "Teleop");
        
        // Cancel autonomous command when entering teleop
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {
    }
    
    @Override
    public void teleopExit() {
    }
    
    @Override
    public void testInit() {
        Logger.recordOutput("RobotMode", "Test");
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {
    }
    
    @Override
    public void testExit() {
    }
    
    @Override
    public void simulationInit() {
        Logger.recordOutput("SimulationInit", true);
    }
    
    @Override
    public void simulationPeriodic() {
        // Simulation-specific updates can go here
    }
}
