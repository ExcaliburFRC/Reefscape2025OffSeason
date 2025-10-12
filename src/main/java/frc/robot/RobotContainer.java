// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.slam.mapper.AuroraClient;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.Automations;
import frc.robot.util.AlgaeScoreState;
import frc.robot.util.CoralScoreState;
import monologue.Logged;

import static frc.robot.Constants.AURORA_CLIENT_PORT;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static monologue.Annotations.*;
import static monologue.Annotations.Log.*;


public class RobotContainer implements Logged {

    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);
    CommandPS5Controller operator = new CommandPS5Controller(1);

    AuroraClient client = new AuroraClient(AURORA_CLIENT_PORT);

    Superstructure superstructure;

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    boolean coralFlag = false;
    Trigger virtualCoralButton = new Trigger(() -> DriverStation.isAutonomous() && coralFlag);

    ClimberSubsystem climber = new ClimberSubsystem();

    Automations automations = new Automations(swerve);


    public RobotContainer() {
        superstructure = new Superstructure(
                new Trigger(() -> swerve.isAtPosition()),
                driver.L1(),
                driver.R1().or(virtualCoralButton),
                new Trigger(() -> swerve.getPose2D().getTranslation().getDistance(AllianceUtils.getReefCenter()) > 2.13456),
                new Trigger(() -> automations.atL2Slice()),
                new Trigger(() -> automations.isLeftReefScore()),
                driver.povLeft()
        );
        configureBindings();
    }

    private void configureBindings() {
        driver.R2().whileTrue(automations.alignToSide(true));
        driver.L2().whileTrue(automations.alignToSide(false));

        driver.triangle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L4));
        driver.circle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L3));
        driver.square().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L2));
        driver.cross().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L1));

        driver.povUp().onTrue(superstructure.setAlgaeScoreStateCommand(AlgaeScoreState.NET));

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-driver.getLeftY()) * MAX_VEL,
                                applyDeadband(-driver.getLeftX()) * MAX_VEL),
                        () -> applyDeadband(-driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

//        driver.povUp().toggleOnTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())));

        driver.touchpad().whileTrue(superstructure.elevatorSubsystem.coastCommand().ignoringDisable(true));
        driver.options().toggleOnTrue(superstructure.intakeSubsystem.resetAngleCommand().ignoringDisable(true));
        driver.create().onTrue(superstructure.elevatorSubsystem.setElevatorHeightCommand(0.16).ignoringDisable(true));

        climber.setDefaultCommand(
                climber.manualCommand(
                        () -> operator.getLeftY(),
                        () -> operator.getRightY()*6)
        );

        operator.triangle().onTrue(superstructure.setCurrentStateCommand(RobotState.CLIMB));
    }

    public void perodic() {
        if (!client.getPose2d().equals(new Pose2d())) {
            swerve.m_odometry.addVisionMeasurement(client.getPose2d(), Timer.getFPGATimestamp());
        }
    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }

    public Command pressVirtualButton() {
        return new InstantCommand(() -> coralFlag = true);
    }

    public Command releaseVirtualButton() {
        return new InstantCommand(() -> coralFlag = false);
    }

    public Command getAutonomousCommand() {
        Command auto = swerve.driveCommand(
                        () -> new Vector2D(2, 0),
                        () -> 0,
                        () -> false)
                .withTimeout(2.7).andThen(
                        superstructure.setCoralScoreStateCommand(CoralScoreState.L1)
                ).andThen(pressVirtualButton()
                ).andThen(releaseVirtualButton()
                ).andThen(new WaitUntilCommand(superstructure.atPositionTrigger.and(() -> superstructure.getCurrentState().equals(RobotState.PRE_L1)))
                ).andThen(pressVirtualButton()
                ).andThen(new WaitCommand(0.5)
                ).andThen(releaseVirtualButton());
//                .andThen(new InstantCommand(() -> flag = true))
//                .andThen(new InstantCommand(() -> flag = true)).andThen(superstructure.getCurrentProcessSupplier().equals())
//                .andThen(new InstantCommand(() -> flag = true));
        return auto;
    }

    @NT
    public Pose2d getRobotPose() {
        return swerve.getPose2D();
    }

    @NT
    public Pose2d getAuroraPose() {
        return client.getPose2d();
    }

    @Log.NT
    public boolean r2() {
        return driver.R2().getAsBoolean();
    }

    @Log.NT
    public double getSupposedClimberHeight() {
        return 0.063;
    }

    @Log.NT
    public double getSupposedOpenClimberHeight() {
        return -Math.PI/2;
    }
}
