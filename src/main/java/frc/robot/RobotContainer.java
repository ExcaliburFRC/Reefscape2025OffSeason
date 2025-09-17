// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.excalib.control.math.Vector2D;
import frc.excalib.swerve.Swerve;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import monologue.Logged;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.SwerveConstants.MAX_OMEGA_RAD_PER_SEC;
import static frc.robot.Constants.SwerveConstants.MAX_VEL;


public class RobotContainer implements Logged {
    CommandPS5Controller driver = new CommandPS5Controller(DRIVER_CONTROLLER_PORT);
    //    IntakeTest test = new IntakeTest();
    ArmSubsystem arm = new ArmSubsystem();
    //    AuroraClient client = new AuroraClient(5000);
//    Intake intake = new Intake(IntakeState.DEFAULT);

        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
//    Swerve swerve = Constants.SwerveConstants.configureSwerve(new Pose2d());


    public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {

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

        driver.square().toggleOnTrue(arm.setStateCommand(ArmPosition.CHECK1).andThen(arm.goToStateCommand()));
//        driver.triangle().toggleOnTrue(arm.setStateCommand(ArmPosition.CHECK2));
       driver.povUp().toggleOnTrue(elevatorSubsystem.setStateCommand(ElevatorStates.L3));
//       driver.povDown().toggleOnTrue(elevatorSubsystem.setStateCommand(ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE));

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
