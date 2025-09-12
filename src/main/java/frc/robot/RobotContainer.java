// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;


public class RobotContainer implements Logged {
    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
//    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {
//        swerve.setDefaultCommand(
//                swerve.driveCommand(
//                        () -> new Vector2D(
//                                -applyDeadband(driver.getLeftY() * MAX_VEL),// * controllerInterpolation.get(controller.getRawAxis(3)),
//                                -applyDeadband(driver.getLeftX() * MAX_VEL)),
//                        () -> -applyDeadband(driver.getRightX() * MAX_OMEGA_RAD_PER_SEC),
//                        () -> true
//                )
//        );
        driver.povDown().whileTrue(elevatorSubsystem.manualCommand(()->-0.1));
        driver.povUp().whileTrue(elevatorSubsystem.manualCommand(()->0.1));
//
//        driver.R1().whileTrue(automations.alignToSide(true));
//        driver.L1().whileTrue(automations.alignToSide(false));
//
//        driver.triangle().onTrue(automations.L4Command());
//        driver.circle().onTrue(automations.L3Command());
//        driver.square().onTrue(automations.L2Command());
//        driver.cross().onTrue(automations.L1Command());
//
//        driver.create().toggleOnTrue(automations.climbOnSelected());
//
//        operator.povLeft().onTrue(new RunCommand(() -> automations.climbOperator.goToNext()));
//        operator.povRight().onTrue(new RunCommand(() -> automations.climbOperator.goToPrev()));

    }

    public double applyDeadband(double val) {
        return val < 0.05 ? 0 : val;
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
