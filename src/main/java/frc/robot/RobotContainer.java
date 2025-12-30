// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.AllianceUtils;
import frc.excalib.control.math.Vector2D;
import frc.excalib.slam.mapper.AuroraClient;
import frc.excalib.swerve.Swerve;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.automations.Automations;
import frc.robot.util.CoralScoreState;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.AURORA_CLIENT_PORT;
import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;
import static monologue.Annotations.*;
import static monologue.Annotations.Log.*;


public class RobotContainer implements Logged {

    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);

    AuroraClient client = new AuroraClient(AURORA_CLIENT_PORT);

//    Superstructure superstructure;

    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());

    Automations automations = new Automations(swerve);

    NetworkTable auroraTable = NetworkTableInstance.getDefault().getTable("Aurora");
    DoubleSupplier x = ()-> 0 , y=()-> 0 , z=()-> 0 , roll=()-> 0, pitch=()-> 0, yaw=()-> 0;



    public RobotContainer() {
//        superstructure = new Superstructure(
//                new Trigger(() -> swerve.isAtPosition()),
//                driver.L1(),
//                driver.R1(),
//                new Trigger(() -> swerve.getPose2D().getTranslation().getDistance(AllianceUtils.getReefCenter()) > 2.13456),
//                new Trigger(() -> automations.atL2Slice()),
//                new Trigger(() -> automations.isLeftReefScore()),
//                driver.povLeft(),
//                driver.PS()
//       );
        configureBindings();
    }


    private void configureBindings() {
//        driver.triangle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L4));
//        driver.circle().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L3));
//        driver.square().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L2));
//        driver.cross().onTrue(superstructure.setCoralScoreStateCommand(CoralScoreState.L1));

        swerve.setDefaultCommand(
                swerve.driveCommand(
                        () -> new Vector2D(
                                applyDeadband(-driver.getLeftY()) * MAX_VEL,
                                applyDeadband(-driver.getLeftX()) * MAX_VEL),
                        () -> applyDeadband(-driver.getRightX()) * MAX_OMEGA_RAD_PER_SEC,
                        () -> true
                )
        );

        driver.povUp().toggleOnTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())));

//        driver.touchpad().whileTrue(superstructure.elevatorSubsystem.coastCommand().ignoringDisable(true));
//
//        driver.options().toggleOnTrue(superstructure.intakeSubsystem.resetAngleCommand().ignoringDisable(true));
//        driver.create().onTrue(superstructure.elevatorSubsystem.setElevatorHeightCommand(0.16).ignoringDisable(true));

//        climber.setDefaultCommand(
//                climber.manualCommand(
//                        () -> operator.getLeftY(),
//                        () -> operator.getRightY()*6)
//        );
//
//        operator.triangle().onTrue(superstructure.setCurrentStateCommand(RobotState.CLIMB));
    }

    public void perodic() {

        x = () -> auroraTable.getEntry("robotPose/x").getDouble(0.0);
        y = () -> auroraTable.getEntry("robotPose/y").getDouble(0.0);
        z = () -> auroraTable.getEntry("robotPose/z").getDouble(0.0);

        yaw = () -> auroraTable.getEntry("robotPose/yaw").getDouble(0.0);
        roll = () -> auroraTable.getEntry("robotPose/roll").getDouble(0.0);
        pitch = () -> auroraTable.getEntry("robotPose/pitch").getDouble(0.0);

    }

    public double applyDeadband(double val) {
        return Math.abs(val) < 0.09 ? 0 : val;
    }


    public Command getAutonomousCommand() {

        return Commands.none();
    }

    @NT
    public Pose2d getRobotPose() {
        return new Pose2d(
                new Translation2d(x.getAsDouble(), y.getAsDouble()),
                new Rotation2d(yaw.getAsDouble())
        );
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
        return -Math.PI / 2;
    }

}
