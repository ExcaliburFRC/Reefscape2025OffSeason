package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
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
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean()
                        && armSubsystem.isAtPosition().getAsBoolean()
                        && intakeSubsystem.isAtPosition().getAsBoolean());
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngle(armSubsystem.getAngleSupplier());
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem.getElevatorHeight());
        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());
    }

    public Command setCurrentStateCommand(RobotStates state) {
        return new InstantCommand(() -> this.currentState = state);
//        new WaitUntilCommand(atPositionTrigger); // TODO: sequsudghsfhntial command
    }

    public void returnToDefaultState() {
        this.currentState = RobotStates.DEFAULT;
    }

    public RobotStates findFollowthroughState(RobotStates scoreState) throws IllegalArgumentException {
        switch (scoreState) {
            case L2 -> {
                return RobotStates.L2_FOLLOWTHROUGH;
            }
            case L3 -> {
                return RobotStates.L3_FOLLOWTHROUGH;
            }
            case L4 -> {
                return RobotStates.L4_FOLLOWTHROUGH;
            }
            default -> {
                throw new IllegalArgumentException("Please enter a valid Robot State");
            }
        }
    }


    public Command reefScoreCommand(RobotStates scoreState) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(scoreState),
                        new WaitUntilCommand(atPositionTrigger),
                        new ParallelCommandGroup(
                                gripperSubsystem.releaseCoral(),
                                setCurrentStateCommand(findFollowthroughState(scoreState))
                        ),
                        setCurrentStateCommand(RobotStates.DEFAULT)),
                new PrintCommand("There is no available coral to score."),
                intakeSubsystem.hasCoral);
    }

    public Command intakeCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.FLOOR_INTAKE),
                        new WaitUntilCommand(intakeSubsystem.hasCoral),
                        setCurrentStateCommand(RobotStates.DEFAULT)),
                new PrintCommand("There is already a coral in the system"),
                intakeSubsystem.hasCoral);
    }
}
