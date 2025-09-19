// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.climber.ClimberSubsystem;
import monologue.Logged;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;


public class RobotContainer implements Logged {
    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    //    IntakeTest test = new IntakeTest();
//    Superstructure superstructure = new Superstructure();
    //    AuroraClient client = new AuroraClient(5000);
//    Intake intake = new Intake(IntakeState.DEFAULT);

//    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
//    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());


    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

        driver.povUp().whileTrue(climberSubsystem.manualCommand(()-> 0.5));
        driver.povDown().whileTrue(climberSubsystem.manualCommand(()-> -0.5));
        driver.triangle().whileTrue(climberSubsystem.manualCommand(()-> 0.1));
        driver.circle().whileTrue(climberSubsystem.manualCommand(()-> -0.1));
//        driver.square().toggleOnTrue(superstructure.handoffCommand());

//        swerve.setDefaultCommand(
//                swerve.driveCommand(
//                        () -> new Vector2D(
//                                -driver.getLeftY() * MAX_VEL,
//                                (-driver.getLeftX()) * MAX_VEL),
//                        () -> driver.getRightX() * MAX_OMEGA_RAD_PER_SEC,
//                        () -> true
//                )
//        );

//        driver.square().toggleOnTrue(swerve.driveSysId(0, SysIdRoutine.Direction.kReverse, new SysidConfig(1,3,8), false));
//        driver.triangle().toggleOnTrue(swerve.driveSysId(0, SysIdRoutine.Direction.kForward, new SysidConfig(1,3,8), false));
//        driver.cross().toggleOnTrue(swerve.driveSysId(0, SysIdRoutine.Direction.kReverse, new SysidConfig(1,3,8), true));
//        driver.circle().toggleOnTrue(swerve.driveSysId(0, SysIdRoutine.Direction.kForward, new SysidConfig(1,3,8), true));
//
//        driver.povUp().toggleOnTrue(swerve.m_MODULES.m_frontLeft.m_driveWheel.setDynamicVelocityCommand(()-> Math.PI));
//        driver.povDown().toggleOnTrue(swerve.m_MODULES.m_frontLeft.driveWheel.setDynamicVelocityCommand(()-> -Math.PI));

//        driver.square().toggleOnTrue(arm.setStateCommand(ArmPosition.CHECK1).andThen(arm.goToStateCommand()));
//        driver.triangle().toggleOnTrue(arm.setStateCommand(ArmPosition.CHECK2));
//        driver.povUp().toggleOnTrue(elevatorSubsystem.setStateCommand(ElevatorStates.L3));
//       driver.povDown().toggleOnTrue(elevatorSubsystem.setStateCommand(ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE));

//        driver.square().toggleOnTrue(superstructure.elevatorSubsystem);
//        driver.square().toggleOnTrue(superstructure.armSubsystem.setStateCommand(ArmPosition.L2));
//        driver.circle().toggleOnTrue(superstructure.gripperSubsystem.releaseCoral());
//        driver.triangle().toggleOnTrue(superstructure.armSubsystem.setStateCommand(ArmPosition.DEFAULT_WITHOUT_GAME_PIECE));

//        driver.triangle().toggleOnTrue(superstructure.intakeCommand());
//        driver.create().toggleOnTrue(superstructure.elevatorSubsystem);

//        driver.options().onTrue(swerve.resetAngleCommand());
    }

    public double applyDeadband(double val) {
        return val < 0.05 ? 0 : val;
    }




    public Command getAutonomousCommand() {
        return Commands.none();
    }

//    @NT
//    public Pose3d getRobotPose() {
//        return client.getPose3d();
//    }
}
