package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.commands.MapCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

import java.util.HashMap;

import static frc.robot.superstructure.Constants.L1_SCORE_VOLTAGE;

public class Superstructure {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Intake intakeSubsystem;
    private final Gripper gripperSubsystem;
    private final Trigger atPositionTrigger;
    private final HashMap<RobotStates, RobotStates> followThroughMap;
    private RobotStates currentState;

    public Superstructure() {
        currentState = RobotStates.DEFAULT;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake(IntakeState.STOW);
        followThroughMap = new HashMap<RobotStates, RobotStates>();
        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean()
                        && armSubsystem.isAtPosition().getAsBoolean()
                        && intakeSubsystem.isAtPosition().getAsBoolean());
        gripperSubsystem = new Gripper();
        elevatorSubsystem.setArmAngle(armSubsystem.getAngleSupplier());
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem.getElevatorHeight());

        armSubsystem.setIntakeOpen(intakeSubsystem.isIntakeOpen());
        followThroughMap.put(RobotStates.L2, RobotStates.L2_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L3, RobotStates.L3_FOLLOWTHROUGH);
        followThroughMap.put(RobotStates.L4, RobotStates.L4_FOLLOWTHROUGH);

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
                                setCurrentStateCommand(followThroughMap.get(scoreState))
                        ),
                        setCurrentStateCommand(RobotStates.DEFAULT)),
                new PrintCommand("There is no available coral to score."),
                intakeSubsystem.hasCoral);
    }

    public Command L1ScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.L1).until(atPositionTrigger),
                        intakeSubsystem.setRollerVoltage(L1_SCORE_VOLTAGE).until(intakeSubsystem.hasCoral.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT)
                ),
                new PrintCommand("There is no available coral to score."),
                intakeSubsystem.hasCoral);
    }

    public Command intakeCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.FLOOR_INTAKE).until(atPositionTrigger),
                        new WaitUntilCommand(intakeSubsystem.hasCoral),
                        setCurrentStateCommand(RobotStates.DEFAULT)),
                new PrintCommand("There is already a coral in the system"),
                intakeSubsystem.hasCoral);
    }

    public Command handoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PREHANDOFF),
                        new WaitUntilCommand(atPositionTrigger),
                        gripperSubsystem.intakeCoral().alongWith(
                                setCurrentStateCommand(RobotStates.HANDOFF)
                        )).until(gripperSubsystem.m_hasCoralTrigger),
                new PrintCommand("There is no a coral in the system"),
                intakeSubsystem.hasCoral.and(gripperSubsystem.m_hasAlgaeTrigger.negate()).and(gripperSubsystem.m_hasCoralTrigger.negate()));
    }

    public Command ejectCoralCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.EJECT_CORAL).until(atPositionTrigger.and(intakeSubsystem.hasCoral.negate())),
                        setCurrentStateCommand(RobotStates.DEFAULT)
                ),
                new PrintCommand("There is no a coral in the system"),
                intakeSubsystem.hasCoral);
    }

    public Command netScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.NET).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.m_hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.m_hasAlgaeTrigger);
    }

    public Command processorScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PROCESSOR).until(atPositionTrigger),
                        new WaitUntilCommand(gripperSubsystem.m_hasAlgaeTrigger.negate()),
                        gripperSubsystem.releaseAlgae(),
                        setCurrentStateCommand(RobotStates.DEFAULT)
                ),
                new PrintCommand("There is no available algae to score."),
                gripperSubsystem.m_hasAlgaeTrigger);
    }

    public Command reverseHandoff() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.PREHANDOFF),
                        gripperSubsystem.releaseCoral(),
                        new WaitUntilCommand(gripperSubsystem.m_hasCoralTrigger.negate().and(intakeSubsystem.hasCoral)),
                        setCurrentStateCommand(RobotStates.DEFAULT)
                ),
                new PrintCommand("There is no available coral to reverse handoff"),
                gripperSubsystem.m_hasAlgaeTrigger);
    }

}

// algae intake & release | reverse handoff |
