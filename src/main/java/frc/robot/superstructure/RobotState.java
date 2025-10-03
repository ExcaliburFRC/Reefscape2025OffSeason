package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.gripper.GripperStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotState {
    PRE_L2(ArmPosition.PRE_L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.VACENT),
    SCORE_L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.VACENT),
    POST_L2(ArmPosition.L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    PRE_L3(ArmPosition.PRE_L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.VACENT),
    SCORE_L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.VACENT),
    POST_L3(ArmPosition.L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    PRE_L4(ArmPosition.PRE_L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.VACENT),
    SCORE_L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.VACENT),
    POST_L4(ArmPosition.L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    ALGAE2_INTAKE(ArmPosition.INTAKE_ALGAE, ElevatorStates.ALGAE2, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    ALGAE3_INTAKE(ArmPosition.INTAKE_ALGAE, ElevatorStates.ALGAE3, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    PRE_PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    NET_SCORE_STAGE_1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.ALGAE),
    NET_SCORE_STAGE_2(ArmPosition.PRE_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.ALGAE),
    NET_SCORE_STAGE_3(ArmPosition.SCORE_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.RELEASE_ALGAE),
    NET_SCORE_STAGE_4(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.VACENT),

    L1_DEFAULT(ArmPosition.DOWNWARDS, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.DEFAULT, GripperStates.VACENT),
    PRE_L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_CORAL, IntakeState.L1_SCORE_PRE, GripperStates.VACENT),
    SCORE_L1(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_CORAL, IntakeState.L1_SCORE, GripperStates.VACENT),
    PRE_L1_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_ALGAE, IntakeState.L1_SCORE, GripperStates.INTAKE_CORAL),
    SCORE_L1_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_ALGAE, IntakeState.L1_SCORE, GripperStates.INTAKE_CORAL),

    FLOOR_INTAKE(ArmPosition.DOWNWARDS, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.FLOOR_INTAKE, GripperStates.INTAKE_CORAL),
    FLOOR_INTAKE_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.FLOOR_INTAKE, GripperStates.ALGAE),

    CENTERLIZE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.CENTERLIZE, GripperStates.VACENT),
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DOWNWARDS, ElevatorStates.DEFAULT_WITHOUT_GAME_PIECE, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),

    DEFAULT_WITH_CORAL(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_CORAL, IntakeState.DEFAULT, GripperStates.VACENT),
    DEFAULT_WITH_ALGAE(ArmPosition.DEFAULT_WITH_GAME_PIECE, ElevatorStates.DEFAULT_WITH_ALGAE, IntakeState.DEFAULT, GripperStates.VACENT),

    PRE_HANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF, IntakeState.DEFAULT, GripperStates.INTAKE_CORAL),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL),
    PRE_REVERSE_HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.REVERSE_HANDOFF, GripperStates.CORAL),

    REVERSE_HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.REVERSE_HANDOFF, GripperStates.RELEASE_CORAL),
    CLIMB(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL);

    ArmPosition armPosition;
    ElevatorStates elevatorState;
    IntakeState intakeState;
    GripperStates gripperState;

    RobotState(ArmPosition armPosition, ElevatorStates elevatorState, IntakeState intakeState, GripperStates gripperState) {
        this.armPosition = armPosition;
        this.elevatorState = elevatorState;
        this.intakeState = intakeState;
        this.gripperState = gripperState;
    }
}
