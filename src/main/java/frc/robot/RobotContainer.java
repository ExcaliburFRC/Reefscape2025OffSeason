// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.slam.mapper.AuroraClient;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.superstructure.Superstructure;
import monologue.Logged;

import static frc.robot.Constants.AURORA_CLIENT_PORT;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static monologue.Annotations.Log.*;


public class RobotContainer implements Logged {

    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    AuroraClient client = new AuroraClient(AURORA_CLIENT_PORT);

    Superstructure superstructure = new Superstructure();
//    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

//    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

//        swerve.setDefaultCommand(
//                swerve.driveCommand(
//                        () -> new Vector2D(
//                                applyDeadband(-driver.getLeftY()) * MAX_VEL,
//                                applyDeadband(-driver.getLeftX()) * MAX_VEL),
//                        () -> applyDeadband(driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
//                        () -> true
//                )
//        );

        driver.L1().onTrue(superstructure.intakeCommand());
        driver.R1().onTrue(superstructure.handoffCommand());

        driver.cross().onTrue(superstructure.L2ScoreCommand());
        driver.circle().onTrue(superstructure.L3ScoreCommand());

        driver.povDown().onTrue(superstructure.returnToDefaultCommand().ignoringDisable(true));
        driver.create().whileTrue(superstructure.intakeSubsystem.resetAngleCommand().ignoringDisable(true));
        driver.square().whileTrue(superstructure.elevatorSubsystem.coastCommand().ignoringDisable(true));
        driver.options().whileTrue(superstructure.elevatorSubsystem.setElevatorHeightCommand(0).ignoringDisable(true));


//        driver.povUp().onTrue(superstructure.L1ScoreCommand());
//        driver.options().whileTrue(superstructure.elevatorSubsystem.coastCommand().ignoringDisable(true));

//        driver.create().onTrue(superstructure.elevatorSubsystem.setElevatorHeightCommand(0).ignoringDisable(true));
    }

    public double applyDeadband(double val) {
        return val < 0.05 ? 0 : val;
    }


    public Command getAutonomousCommand() {
        return Commands.none();
    }

    @NT
    public Pose2d getRobotPose() {
        return client.getPose2d();
    }
}
