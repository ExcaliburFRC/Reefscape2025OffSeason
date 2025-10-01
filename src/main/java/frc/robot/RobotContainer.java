// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.slam.mapper.AuroraClient;
import frc.excalib.slam.mapper.PoseEstimator;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.Automations;
import frc.robot.util.CoralScoreState;
import monologue.Logged;

import static frc.robot.Constants.AURORA_CLIENT_PORT;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static monologue.Annotations.Log.*;


public class RobotContainer implements Logged {

    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    AuroraClient client = new AuroraClient(AURORA_CLIENT_PORT);

//    Superstructure superstructure = new Superstructure(new Trigger(() -> true), driver.R2());

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    Automations automations = new Automations(swerve);


    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

//        driver.R1().onTrue(superstructure.setCurrentProcessCommand(Superstructure.Process.SCORE_CORAL));
//        driver.triangle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L4));
//        driver.circle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L3));
//        driver.square().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L2));
//        driver.PS().onTrue(superstructure.setCurrentProcessCommand(Superstructure.Process.DEFAULT));
//        driver.L1().onTrue(superstructure.setCurrentProcessCommand(Superstructure.Process.INTAKE_CORAL));

//        driver.triangle().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d(Constants.FieldConstants.B1_LEFT_SCORE, Rotation2d.k180deg))).ignoringDisable(true));
//        driver.povLeft().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d(Constants.FieldConstants.B12_LEFT_SCORE, Rotation2d.k180deg))).ignoringDisable(true));
//        driver.circle().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d(AllianceUtils.getReefCenter(), Rotation2d.kZero))).ignoringDisable(true));
//        driver.square().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d(16, 2, Rotation2d.kZero))).ignoringDisable(true));
        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-driver.getLeftY()) * MAX_VEL,
                                applyDeadband(-driver.getLeftX()) * MAX_VEL),
                        () -> applyDeadband(driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

        driver.options().whileTrue(new RunCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.kZero))).ignoringDisable(true));

//        driver.touchpad().whileTrue(superstructure.elevatorSubsystem.coastCommand().ignoringDisable(true));

//        driver.povDown().toggleOnTrue(superstructure.intakeSubsystem.resetAngleCommand().ignoringDisable(true));
//        driver.create().onTrue(superstructure.elevatorSubsystem.setElevatorHeightCommand(0.15).ignoringDisable(true));

    }

    public void preodic() {
        if (!client.getPose2d().equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(client.getPose2d(), Timer.getFPGATimestamp());
        }
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }


    public Command getAutonomousCommand() {
        return Commands.none();
    }

    @NT
    public Pose2d getRobotPose() {
        return swerve.getPose2D();
    }

    @NT
    public Pose2d getAuroraPose() {
        return client.getPose2d();
    }

}
