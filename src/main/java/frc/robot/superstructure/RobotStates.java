package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    // ------ HIGH REEF CORAL SCORE ------
    L2_FOLLOWTHROUGH(ArmPosition.L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT),
    L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.DEFAULT),
    L3_FOLLOWTHROUGH(ArmPosition.L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT),
    L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.DEFAULT),
    L4_FOLLOWTHROUGH(ArmPosition.L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT),

    // ------ ALGAE REMOVAL & SCORE ------
    ALGAE2(ArmPosition.ALGAE2, ElevatorStates.ALGAE2, IntakeState.DEFAULT),
    ALGAE3(ArmPosition.ALGAE3, ElevatorStates.ALGAE3, IntakeState.DEFAULT),
    NET(ArmPosition.NET, ElevatorStates.NET, IntakeState.DEFAULT),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT),


    // ------ INTAKE ------
    L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE),
    L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.DEFAULT),
    FLOOR_INTAKE(ArmPosition.DEFAULT_WITHOUT_GAME_PIECE, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.FLOOR_INTAKE),
    EJECT_CORAL_FROM_INTAKE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.EJECT_CORAL),
    EJECT_GAME_PIECE_FROM_GRIPPER(ArmPosition.EJECT_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT),


    // ------ GENERAL ------
    DEFAULT_WITH_GAME_PIECE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT),
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DEFAULT_WITHOUT_GAME_PIECE, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.DEFAULT),
    PREHANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF , IntakeState.HANDOFF),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF);


    ArmPosition armPosition;
    ElevatorStates elevatorState;
    IntakeState intakeState;

    RobotStates(ArmPosition armPosition, ElevatorStates elevatorState, IntakeState intakeState){
        this.armPosition = armPosition;
        this.elevatorState = elevatorState;
        this.intakeState = intakeState;
    }
}
