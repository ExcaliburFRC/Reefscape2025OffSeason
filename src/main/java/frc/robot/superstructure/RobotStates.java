package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    // ------ HIGH REEF CORAL SCORE ------
    L2_FOLLOWTHROUGH(ArmPosition.L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.STOW),
    L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.DEFAULT, ClimberStates.STOW),
    L3_FOLLOWTHROUGH(ArmPosition.L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.STOW),
    L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.DEFAULT, ClimberStates.STOW),
    L4_FOLLOWTHROUGH(ArmPosition.L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.STOW),

    // ------ ALGAE REMOVAL & SCORE ------
    ALGAE2(ArmPosition.ALGAE2, ElevatorStates.ALGAE2, IntakeState.DEFAULT, ClimberStates.STOW),
    ALGAE3(ArmPosition.ALGAE3, ElevatorStates.ALGAE3, IntakeState.DEFAULT, ClimberStates.STOW),
    NET(ArmPosition.NET, ElevatorStates.NET, IntakeState.DEFAULT, ClimberStates.STOW),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, ClimberStates.STOW),


    // ------ INTAKE ------
    L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE, ClimberStates.STOW),
    L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.DEFAULT, ClimberStates.STOW),
    FLOOR_INTAKE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.FLOOR_INTAKE, ClimberStates.STOW),
    EJECT_CORAL_FROM_INTAKE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.EJECT_CORAL, ClimberStates.STOW),
    EJECT_GAME_PIECE_FROM_GRIPPER(ArmPosition.EJECT_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, ClimberStates.STOW),


    // ------ GENERAL ------
    DEFAULT_WITH_GAME_PIECE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, ClimberStates.STOW),
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.DEFAULT, ClimberStates.STOW),
    PREHANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF , IntakeState.HANDOFF, ClimberStates.STOW),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, ClimberStates.STOW),
    PRE_CLIMB(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.DEFAULT, ClimberStates.OPEN);


    ArmPosition armPosition;
    ElevatorStates elevatorState;
    ClimberStates climberStates;
    IntakeState intakeState;

    RobotStates(ArmPosition armPosition, ElevatorStates elevatorState, IntakeState intakeState, ClimberStates aDefault){
        this.armPosition = armPosition;
        this.elevatorState = elevatorState;
        this.intakeState = intakeState;
    }
}
