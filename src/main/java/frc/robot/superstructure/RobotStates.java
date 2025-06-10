package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    L1(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.L1_SCORE),
    L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.STOW),
    L2_FOLLOWTHROUGH(ArmPosition.L2_FOLLOWTHROUGH, ElevatorStates.L2_FOLLOWTHROUGH, IntakeState.STOW),
    L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.STOW),
    L3_FOLLOWTHROUGH(ArmPosition.L3_FOLLOWTHROUGH, ElevatorStates.L3_FOLLOWTHROUGH, IntakeState.STOW),
    L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.STOW),
    L4_FOLLOWTHROUGH(ArmPosition.L4_FOLLOWTHROUGH, ElevatorStates.L4_FOLLOWTHROUGH, IntakeState.STOW),
    ALGAE2(ArmPosition.ALGAE2, ElevatorStates.ALGAE2, IntakeState.STOW),
    ALGAE3(ArmPosition.ALGAE3, ElevatorStates.ALGAE3, IntakeState.STOW),
    FLOOR_INTAKE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.FLOOR_INTAKE),

    DEFAULT(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.STOW),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF),
    PREHANDOFF(ArmPosition.HANDOFF, ElevatorStates.PRE_HANDOFF , IntakeState.HANDOFF),
    NET(ArmPosition.NET, ElevatorStates.NET, IntakeState.STOW),
    PROCESSOR(ArmPosition.PROCESSOR, ElevatorStates.PROCESSOR, IntakeState.STOW);

    ArmPosition armPosition;
    ElevatorStates elevatorState;
    IntakeState intakeState;

    RobotStates(ArmPosition armPosition, ElevatorStates elevatorState, IntakeState intakeState){
        this.armPosition = armPosition;
        this.elevatorState = elevatorState;
        this.intakeState = intakeState;
    }
}
