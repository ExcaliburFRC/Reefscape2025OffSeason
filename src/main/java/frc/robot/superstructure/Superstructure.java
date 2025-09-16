package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import static frc.robot.superstructure.automations.Constants.AT_POSITION_DEBOUNCE;
import static frc.robot.superstructure.automations.Constants.L1_SCORE_VOLTAGE;

public class Superstructure {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Intake intakeSubsystem;
    private final Gripper gripperSubsystem;
    private final Trigger atPositionTrigger;
    private final HashMap<RobotStates, RobotStates> followThroughMap;
    private RobotStates currentState;

    public Superstructure() {
        currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake(IntakeState.DEFAULT);
        followThroughMap = new HashMap<RobotStates, RobotStates>();
        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean()
                        && armSubsystem.isAtPosition().getAsBoolean()
                        && intakeSubsystem.isAtPosition().getAsBoolean()).debounce(AT_POSITION_DEBOUNCE);
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngle(armSubsystem::getAngleSupplier);
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());
        followThroughMap.put(RobotStates.L2, RobotStates.L2_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L3, RobotStates.L3_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L4, RobotStates.L4_FOLLOWTHROUGH);

    }


    public Command setCurrentStateCommand(RobotStates state) {
        return new InstantCommand(() -> this.currentState = state);
//        new WaitUntilCommand(atPositionTrigger); // TODO: sequntinal command
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
                gripperSubsystem.m_hasCoralTrigger);
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
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.FLOOR_INTAKE).until(atPositionTrigger),
                        new WaitUntilCommand(intakeSubsystem.hasCoral),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)),
                new PrintCommand("There is already a coral in the system"),
                (intakeSubsystem.hasCoral.or(gripperSubsystem.m_hasCoralTrigger)).negate()).withName("intake Command");
    }

    public Command handoffCommand() { //TODO (Yehuda please approve this) make the handoff double sided - from intake to gripper and from gripper to intake depending on the robot's state
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PREHANDOFF),
                        new WaitUntilCommand(atPositionTrigger),
                        gripperSubsystem.intakeCoral().alongWith(
                                setCurrentStateCommand(RobotStates.HANDOFF)
                        )).until(gripperSubsystem.m_hasCoralTrigger),
                new PrintCommand("There is no a coral in the system"),
                intakeSubsystem.hasCoral.and(gripperSubsystem.m_hasAlgaeTrigger.negate()).and(gripperSubsystem.m_hasCoralTrigger.negate())).withName("handoff Command"); //TODO isn't it supposed to be or() instead of and()?
    }

    public Command ejectCommand() { //TODO (needs Yehuda's approval) make the command supercycle friendly
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.EJECT_CORAL_FROM_INTAKE).until(atPositionTrigger.and(intakeSubsystem.hasCoral.negate())),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                setCurrentStateCommand(RobotStates.EJECT_GAME_PIECE_FROM_GRIPPER).until(atPositionTrigger.and((gripperSubsystem.m_hasAlgaeTrigger.or(gripperSubsystem.m_hasCoralTrigger)).negate())),
                                setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                        ),
                        new PrintCommand("There is no a game piece in the system"),
                        gripperSubsystem.m_hasCoralTrigger.or(gripperSubsystem.m_hasAlgaeTrigger)
                ),
                intakeSubsystem.hasCoral).withName("Eject Coral Command");
    }

    public Command netScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.NET).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.m_hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.m_hasAlgaeTrigger).withName("Net Score Command");
    }

    public Command processorScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PROCESSOR).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.m_hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.m_hasAlgaeTrigger).withName("Processor Score Command");
    }

    public Command reverseHandoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PREHANDOFF),
                        gripperSubsystem.releaseCoral(),
                        new WaitUntilCommand(gripperSubsystem.m_hasCoralTrigger.negate().and(intakeSubsystem.hasCoral)),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available coral to reverse handoff"),
                gripperSubsystem.m_hasAlgaeTrigger).withName("Reverse Handoff Command");
    }

    public Command algaeIntakeCommand(RobotStates algaeLevel) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(algaeLevel).until(atPositionTrigger),
                        gripperSubsystem.intakeAlgae().until(gripperSubsystem.m_hasAlgaeTrigger),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
                ),
                new PrintCommand("There is no available place in gripper"),

                (gripperSubsystem.m_hasAlgaeTrigger.or(gripperSubsystem.m_hasCoralTrigger)).negate()).withName("Algae Intake Command");

    }

    public Command algaeReleaseCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        gripperSubsystem.releaseAlgae().until(gripperSubsystem.m_hasAlgaeTrigger.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)
                ),
                new PrintCommand("There is no available algae to release"),
                (gripperSubsystem.m_hasAlgaeTrigger)).withName("Algae Intake Command");
    }

    public Command secureCommand(){
        return Commands.none();
    }

    public BooleanSupplier isAtPosition() {
        return atPositionTrigger;
    }
}


