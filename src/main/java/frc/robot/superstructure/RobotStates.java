package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.gripper.GripperStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    // ------ HIGH REEF CORAL SCORE ------
    RIGHT_PRE_L2(ArmPosition.RIGHT_PRE_L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.VACENT),
    RIGHT_L2_SCORE(ArmPosition.RIGHT_L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),
    RIGHT_L2_POST(ArmPosition.RIGHT_L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),
    RIGHT_PRE_L3(ArmPosition.RIGHT_PRE_L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_L3(ArmPosition.RIGHT_L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_L3_POST(ArmPosition.RIGHT_L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_PRE_L4(ArmPosition.RIGHT_PRE_L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_L4(ArmPosition.RIGHT_L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_L4_FOLLOWTHROUGH(ArmPosition.RIGHT_L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    LEFT_PRE_L2(ArmPosition.LEFT_PRE_L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L2(ArmPosition.LEFT_L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L2_POST(ArmPosition.LEFT_L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_PRE_L3(ArmPosition.LEFT_PRE_L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L3(ArmPosition.LEFT_L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L3_FOLLOWTHROUGH(ArmPosition.LEFT_L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_PRE_L4(ArmPosition.LEFT_PRE_L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L4(ArmPosition.LEFT_L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_L4_FOLLOWTHROUGH(ArmPosition.LEFT_L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    // ------ ALGAE REMOVAL & SCORE ------
    RIGHT_ALGAE2(ArmPosition.RIGHT_ALGAE2, ElevatorStates.ALGAE2, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_ALGAE2(ArmPosition.LEFT_ALGAE2, ElevatorStates.ALGAE2, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    LEFT_ALGAE3(ArmPosition.LEFT_ALGAE3, ElevatorStates.ALGAE3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_ALGAE3(ArmPosition.RIGHT_ALGAE3, ElevatorStates.ALGAE3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    RIGHT_NET(ArmPosition.RIGHT_NET_POST, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    RIGHT_NET_PRE(ArmPosition.RIGHT_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_NET(ArmPosition.LEFT_NET_POST, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_NET_PRE(ArmPosition.LEFT_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    RIGHT_PROCESSOR(ArmPosition.RIGHT_PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    LEFT_PROCESSOR(ArmPosition.LEFT_PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),


    // ------ INTAKE ------
    PRE_L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE_PRE, GripperStates.VACENT),
    SCORE_L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE, GripperStates.VACENT),
    PRE_L1_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE, GripperStates.INTAKE_CORAL),
    SCORE_L1_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.L1_SCORE, GripperStates.INTAKE_CORAL),
    FLOOR_INTAKE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.FLOOR_INTAKE, GripperStates.INTAKE_CORAL),
    FLOOR_INTAKE_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.FLOOR_INTAKE, GripperStates.INTAKE_CORAL),
    EJECT_CORAL_FROM_INTAKE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.EJECT_CORAL, GripperStates.INTAKE_CORAL),
    EJECT_GAME_PIECE_FROM_GRIPPER(ArmPosition.EJECT_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),


    // ------ GENERAL ------
    DEFAULT_WITH_GAME_PIECE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, GripperStates.VACENT),
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DEFAULT_WITHOUT_GAME_PIECE, ElevatorStates.DEFAULT_WITH_GAME_PIECE, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    PRE_HANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL),
    CLIMB(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL);


    ArmPosition armPosition;
    ElevatorStates elevatorState;
    IntakeState intakeState;
    GripperStates gripperState;

    RobotStates(ArmPosition armPosition, ElevatorStates elevatorState, IntakeState intakeState, GripperStates gripperState) {
        this.armPosition = armPosition;
        this.elevatorState = elevatorState;
        this.intakeState = intakeState;
        this.gripperState = gripperState;
    }
}
