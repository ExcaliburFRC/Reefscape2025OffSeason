package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.gripper.GripperStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotState {

    // --- BRANCH SCORING ---
    PRE_L2(ArmPosition.PRE_CORAL_SCORE, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.CORAL),
    SCORE_L2(ArmPosition.TILTED_BRANCH_CORAL_SCORE, ElevatorStates.L2, IntakeState.DEFAULT, GripperStates.CORAL),
    POST_L2(ArmPosition.TILTED_BRANCH_CORAL_SCORE_POST, ElevatorStates.L2_POST, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    PRE_L3(ArmPosition.PRE_CORAL_SCORE, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.CORAL),
    SCORE_L3(ArmPosition.TILTED_BRANCH_CORAL_SCORE, ElevatorStates.L3, IntakeState.DEFAULT, GripperStates.CORAL),
    POST_L3(ArmPosition.TILTED_BRANCH_CORAL_SCORE_POST, ElevatorStates.L3_POST, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    PRE_L4(ArmPosition.PRE_CORAL_SCORE, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.CORAL),
    SCORE_L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.DEFAULT, GripperStates.CORAL),
    POST_L4(ArmPosition.L4_POST, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.DEFAULT, GripperStates.RELEASE_CORAL),

    // L1 SCORING
    L1_DEFAULT(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.DEFAULT, GripperStates.VACENT),
    PRE_L1(ArmPosition.UPWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.L1_SCORE_PRE, GripperStates.VACENT),
    SCORE_L1(ArmPosition.UPWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.L1_SCORE, GripperStates.VACENT),

    // ALGAE INTAKE
    ALGAE2_INTAKE(ArmPosition.INTAKE_ALGAE, ElevatorStates.ALGAE2, IntakeState.DEFAULT, GripperStates.INTAKE_ALGAE),
    ALGAE3_INTAKE(ArmPosition.INTAKE_ALGAE, ElevatorStates.ALGAE3, IntakeState.DEFAULT, GripperStates.INTAKE_ALGAE),

    // ALGAE SCORE
    PRE_PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.ALGAE),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.DEFAULT, GripperStates.RELEASE_ALGAE),

    NET_SCORE_STAGE_1(ArmPosition.UPWARDS, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.ALGAE),
    NET_SCORE_STAGE_2(ArmPosition.PRE_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.ALGAE),
    NET_SCORE_STAGE_3(ArmPosition.SCORE_NET, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.RELEASE_ALGAE),
    NET_SCORE_STAGE_4(ArmPosition.UPWARDS, ElevatorStates.NET, IntakeState.DEFAULT, GripperStates.VACENT),

    // CORAL INTAKE
    FLOOR_INTAKE(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.FLOOR_INTAKE, GripperStates.VACENT),
    FLOOR_INTAKE_WITH_ALGAE(ArmPosition.UPWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.FLOOR_INTAKE, GripperStates.ALGAE),

    // DEFAULTS
    DEFAULT_WITHOUT_GAME_PIECE(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.DEFAULT, GripperStates.VACENT),
    DEFAULT_WITH_CORAL(ArmPosition.UPWARDS, ElevatorStates.DEFAULT_WITH_CORAL, IntakeState.DEFAULT, GripperStates.CORAL),
    DEFAULT_WITH_ALGAE(ArmPosition.UPWARDS, ElevatorStates.DEFAULT_WITH_ALGAE, IntakeState.DEFAULT, GripperStates.ALGAE),

    // HANDOFFS
    PRE_HANDOFF(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.CENTERLIZE, GripperStates.INTAKE_CORAL),
    HANDOFF(ArmPosition.DOWNWARDS, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL),
    PRE_REVERSE_HANDOFF(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.REVERSE_HANDOFF, GripperStates.CORAL),
    REVERSE_HANDOFF(ArmPosition.DOWNWARDS, ElevatorStates.HANDOFF, IntakeState.REVERSE_HANDOFF, GripperStates.RELEASE_CORAL),

    // OTHERS
    CLIMB(ArmPosition.UPWARDS, ElevatorStates.HANDOFF, IntakeState.HANDOFF, GripperStates.INTAKE_CORAL),
    CENTERLIZE(ArmPosition.DOWNWARDS, ElevatorStates.SAFE_HEIGHT, IntakeState.CENTERLIZE, GripperStates.VACENT);

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
