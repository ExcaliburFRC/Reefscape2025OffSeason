package frc.robot.superstructure;

import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeState;

public enum RobotStates {
    L1(ArmPosition.L1, ElevatorStates.DEFAULT, IntakeState.L1_SCORE),
    L2(ArmPosition.L2, ElevatorStates.L2, IntakeState.STOW),
    L3(ArmPosition.L3, ElevatorStates.L3, IntakeState.STOW),
    L4(ArmPosition.L4, ElevatorStates.L4, IntakeState.STOW),
    ALGAE2(ArmPosition.ALGAE2, ElevatorStates.ALGAE2, IntakeState.STOW),
    ALGAE3(ArmPosition.ALGAE3, ElevatorStates.ALGAE3, IntakeState.STOW),
    FLOOR_INTAKE(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.FLOOR_INTAKE),
    STOW(ArmPosition.DEFAULT, ElevatorStates.DEFAULT, IntakeState.STOW),
    HANDOFF(ArmPosition.HANDOFF, ElevatorStates.HANDOFF, IntakeState.HANDOFF),
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
