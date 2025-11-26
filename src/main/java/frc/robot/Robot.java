// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
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

public class Robot extends LoggedRobot implements Logged {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;
    private final RobotContainerWithExamples m_examplesContainer;
    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
    public static boolean isReplay = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
        // Also construct the example container so we can use example subsystems in simulation
        m_examplesContainer = new RobotContainerWithExamples();
        Logger.recordMetadata("Offseason2025", "Offseason2025"); // Set a metadata value

        if (isSimulation() && !isReplay) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else if (isReplay) {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    }

    @Override
    public void robotInit() {
        // RobotContainer::periodic is no longer scheduled via addPeriodic (not available in this LoggedRobot API).
        // Vision and subsystem periodic behavior are handled by the CommandScheduler and subsystem periodic methods.
        CameraServer.startAutomaticCapture();
    }


    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        TalonFXMotor.refreshAll();
        Threads.setCurrentThreadPriority(false, 10);
        // Run RobotContainer periodic BEFORE CommandScheduler so simulation inputs (like IMU yaw)
        // are injected before subsystems' periodic() are run by the scheduler.
        try {
            m_robotContainer.periodic();
        } catch (Exception ignored) {}
        commandScheduler.run();
         // Update monologue (NT/Monologue logging) after commands and subsystems run so it reads latest values
         Monologue.updateAll();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // Use example container's autonomous in simulation (it contains ExampleSwerveSubsystem), otherwise use normal container
        if (isSimulation()) {
            m_autonomousCommand = m_examplesContainer.getAutonomousCommand();
        } else {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        }

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
