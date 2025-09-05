// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.additional_utilities.PS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.Automations;

import static frc.robot.Constants.*;


public class RobotContainer {
    PS5Controller driver = new PS5Controller(DRIVER_CONTROLLER_PORT, true, DRIVER_SIMULATION_CONTROLLER_PORT);
    CommandPS5Controller operator = new CommandPS5Controller(OPERATOR_CONTROLLER_PORT);

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    Superstructure superstructure = new Superstructure();

    Automations automations = new Automations(swerve, superstructure);

    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {
        swerve.applyDriveControllerCommand(driver);

        driver.R1().whileTrue(automations.alignToSide(true));
        driver.L1().whileTrue(automations.alignToSide(false));

        driver.triangle().onTrue(automations.L4Command());
        driver.circle().onTrue(automations.L3Command());
        driver.square().onTrue(automations.L2Command());
        driver.cross().onTrue(automations.L1Command());

        driver.create().toggleOnTrue(automations.climbOnSelected());

        operator.povLeft().onTrue(new RunCommand(() -> automations.climbOperator.goToNext()));
        operator.povRight().onTrue(new RunCommand(() -> automations.climbOperator.goToPrev()));

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
