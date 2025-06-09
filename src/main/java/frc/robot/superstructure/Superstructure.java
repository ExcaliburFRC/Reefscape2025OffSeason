package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class Superstructure {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Intake intakeSubsystem;
    private final Gripper gripperSubsystem;
    private final Trigger atPositionTrigger;
    private RobotStates currentState;

    public Superstructure() {
        currentState = RobotStates.DEFAULT;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
                intakeSubsystem = new Intake(IntakeState.STOW);

        atPositionTrigger = new Trigger(
                ()-> elevatorSubsystem.atPositionTrigger.getAsBoolean()
                        && armSubsystem.isAtPosition().getAsBoolean()
                        && intakeSubsystem.isAtPosition().getAsBoolean());
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngle(armSubsystem.getAngleSupplier());
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem.getElevatorHeight());
        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());
    }

    public Command setCurrentStateCommand(RobotStates state) {
        return new InstantCommand(() -> this.currentState = state);
    }

    public void returnToDefaultState() {
        this.currentState = RobotStates.DEFAULT;
    }

//    public Command L2Command() {
//        return new SequentialCommandGroup(
//                setCurrentStateCommand(RobotStates.L2),
//
//        )
//    }


}
