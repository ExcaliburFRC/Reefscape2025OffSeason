package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import monologue.Logged;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import static frc.robot.superstructure.automations.Constants.AT_POSITION_DEBOUNCE;

public class Superstructure implements Logged {
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final Intake intakeSubsystem;
    public final Gripper gripperSubsystem;
    public final Trigger atPositionTrigger;
    private final HashMap<RobotStates, RobotStates> followThroughMap;
    public RobotStates currentState;

    public Superstructure() {
        currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake(IntakeState.DEFAULT);
        followThroughMap = new HashMap<RobotStates, RobotStates>();
        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean()
                        && armSubsystem.isAtPosition()
                        && intakeSubsystem.isAtPosition().getAsBoolean()).debounce(AT_POSITION_DEBOUNCE);
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngleSuppier(armSubsystem::getAngleSupplier);
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());
        elevatorSubsystem.setIntakeOpenTrigger(new Trigger(intakeSubsystem.isIntakeOpen()));
        followThroughMap.put(RobotStates.L2, RobotStates.L2_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L3, RobotStates.L3_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L4, RobotStates.L4_FOLLOWTHROUGH);

    }

    public Command setCurrentStateCommand(RobotStates state) {
        return new RunCommand(
                () -> new ParallelCommandGroup(
                        intakeSubsystem.setStateCommand(state.intakeState),
                        armSubsystem.setStateCommand(state.armPosition),
                        elevatorSubsystem.setStateCommand(state.elevatorState)
                )
        );
    }

    public void returnToDefaultState() {
        this.currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;
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

    public Command openToScoreCommand(RobotStates scoreState) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(scoreState),
                        new WaitUntilCommand(atPositionTrigger),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)),
                handoffCommand(),
                gripperSubsystem.hasCoralTrigger);
    }

    public Command scoreCommand() {
        return new ParallelCommandGroup(
                gripperSubsystem.releaseCoral(),
                setCurrentStateCommand(followThroughMap.get(currentState))
        );
    }

    public Command L1ScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.L1).until(atPositionTrigger),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available coral to score."), //TODO after handoffComand is changed apply it here like applied at reefScoreCommand
                intakeSubsystem.hasCoral).withName("L1 Score Command");
    }

    public Command intakeCommand() {
        return new SequentialCommandGroup(
                new PrintCommand("1"),
                setCurrentStateCommand(RobotStates.FLOOR_INTAKE).until(intakeSubsystem.isAtPosition()),
                new PrintCommand("2"),
                new WaitUntilCommand(intakeSubsystem::getTriggerData),
                new PrintCommand("3"),
                setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
        );
    }

    public Command handoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        intakeSubsystem.setStateCommand(IntakeState.PREHANDOFF),
                        new PrintCommand("2"),
                        new ParallelCommandGroup(
                                new PrintCommand("3"),
                                intakeSubsystem.goToStateCommand(),
                                armSubsystem.setStateCommand(ArmPosition.HANDOFF),
                                gripperSubsystem.intakeCoral()
                        ),
                        new PrintCommand("4"),
                        new ParallelCommandGroup(
                                intakeSubsystem.setStateCommand(IntakeState.HANDOFF).andThen(intakeSubsystem.goToStateCommand())
                        ).until(gripperSubsystem.hasCoralTrigger),
                        intakeSubsystem.setStateCommand(IntakeState.DEFAULT).andThen(intakeSubsystem.goToStateCommand()).withTimeout(0.1),
                        armSubsystem.setStateCommand(ArmPosition.L3)
                ),
                new PrintCommand("There is no a coral in the system"),
                intakeSubsystem.hasCoral.and(gripperSubsystem.hasAlgaeTrigger.negate()).and(gripperSubsystem.hasCoralTrigger.negate())).withName("handoff Command");
    }

    public Command ejectCommand() { //TODO (needs Yehuda's approval) make the command supercycle friendly
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.EJECT_CORAL_FROM_INTAKE).until(atPositionTrigger.and(intakeSubsystem.hasCoral.negate())),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                setCurrentStateCommand(RobotStates.EJECT_GAME_PIECE_FROM_GRIPPER).until(atPositionTrigger.and((gripperSubsystem.hasAlgaeTrigger.or(gripperSubsystem.hasCoralTrigger)).negate())),
                                setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                        ),
                        new PrintCommand("There is no a game piece in the system"),
                        gripperSubsystem.hasCoralTrigger.or(gripperSubsystem.hasAlgaeTrigger)
                ),
                intakeSubsystem.hasCoral).withName("Eject Coral Command");
    }

    public Command netScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.NET).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.hasAlgaeTrigger).withName("Net Score Command");
    }

    public Command processorScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PROCESSOR).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.hasAlgaeTrigger).withName("Processor Score Command");
    }

    public Command reverseHandoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PREHANDOFF),
                        gripperSubsystem.releaseCoral(),
                        new WaitUntilCommand(gripperSubsystem.hasCoralTrigger.negate().and(intakeSubsystem.hasCoral)),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available coral to reverse handoff"),
                gripperSubsystem.hasAlgaeTrigger).withName("Reverse Handoff Command");
    }

    public Command algaeIntakeCommand(RobotStates algaeLevel) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(algaeLevel).until(atPositionTrigger),
                        gripperSubsystem.intakeAlgae().until(gripperSubsystem.hasAlgaeTrigger),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
                ),
                new PrintCommand("There is no available place in gripper"),

                (gripperSubsystem.hasAlgaeTrigger.or(gripperSubsystem.hasCoralTrigger)).negate()).withName("Algae Intake Command");

    }

    public Command algaeReleaseCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        gripperSubsystem.releaseAlgae().until(gripperSubsystem.hasAlgaeTrigger.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to release"),
                (gripperSubsystem.hasAlgaeTrigger)).withName("Algae Intake Command");
    }

    public Command secureCommand() {
        return Commands.none();
    }

    public BooleanSupplier isAtPosition() {
        return atPositionTrigger;
    }
}


