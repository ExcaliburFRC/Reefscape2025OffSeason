package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    // ------ HIGH REEF CORAL SCORE ------
    L2_FOLLOWTHROUGH(ArmPosition.L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    L3_FOLLOWTHROUGH(ArmPosition.L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    L4_FOLLOWTHROUGH(ArmPosition.L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, ClimberStates.DEFAULT),

    // ------ ALGAE REMOVAL & SCORE ------
    ALGAE2(ArmPosition.ALGAE2, ElevatorStates.ALGAE2, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    ALGAE3(ArmPosition.ALGAE3, ElevatorStates.ALGAE3, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    NET(ArmPosition.NET, ElevatorStates.NET, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, ClimberStates.DEFAULT),


    // ------ INTAKE ------
    L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE, ClimberStates.DEFAULT),
    L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    FLOOR_INTAKE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.FLOOR_INTAKE, ClimberStates.DEFAULT),
    EJECT_CORAL_FROM_INTAKE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.EJECT_CORAL, ClimberStates.DEFAULT),
    EJECT_GAME_PIECE_FROM_GRIPPER(ArmPosition.EJECT_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, ClimberStates.DEFAULT),


    // ------ GENERAL ------
    DEFAULT_WITH_GAME_PIECE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.DEFAULT, ClimberStates.DEFAULT),
    PREHANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF , IntakeState.HANDOFF, ClimberStates.DEFAULT),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, ClimberStates.DEFAULT),
    PRE_CLIMB(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.DEFAULT, ClimberStates.DEFAULT);


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
