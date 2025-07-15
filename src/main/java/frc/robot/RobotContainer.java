// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.Automations;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;


public class RobotContainer {
    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());
    Superstructure superstructure = new Superstructure();

    Automations automations = new Automations(swerve, superstructure);

    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
