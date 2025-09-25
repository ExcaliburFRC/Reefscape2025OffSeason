package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import monologue.Logged;

import java.util.HashMap;

import static monologue.Annotations.*;

public class Superstructure implements Logged {
    // === Subsystems ==
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final Intake intakeSubsystem;
    public final Gripper gripperSubsystem;
    public final ClimberSubsystem climberSubsystem;


    private final HashMap<RobotStates, RobotStates> followThroughMap;
    public RobotStates currentState;
    public final Trigger atPositionTrigger;
    public final Trigger readyToCloseTrigger;


    public Superstructure() {
        currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();
        climberSubsystem = new ClimberSubsystem();

        followThroughMap = new HashMap<>();

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean()
        );

        elevatorSubsystem.setArmAngleSuppier(() -> armSubsystem.getAngleSupplier());
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(new Trigger(intakeSubsystem.isIntakeOpen()));
        elevatorSubsystem.setIntakeOpenTrigger(intakeSubsystem.isIntakeOpen());

//        followThroughMap.put(RobotStates.L2, RobotStates.L2_FOLLOWTHROUGH);
//        followThroughMap.put(RobotStates.L3, RobotStates.L3_FOLLOWTHROUGH);
//        followThroughMap.put(RobotStates.L4, RobotStates.L4_FOLLOWTHROUGH);

        readyToCloseTrigger = new Trigger(atPositionTrigger.and(intakeSubsystem.hasCoral));

    }

    public Command setCurrentStateCommand(RobotStates state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentState = state),
                new PrintCommand("changed state"),
                intakeSubsystem.setStateCommand(state.intakeState),
                armSubsystem.setStateCommand(state.armPosition),
                elevatorSubsystem.setStateCommand(state.elevatorState),
                gripperSubsystem.setStateCommand(state.gripperState)
        ).until(atPositionTrigger);
    }

    public void returnToDefaultState() {
        this.currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;
    }

    public RobotStates findFollowthroughState(RobotStates scoreState) throws IllegalArgumentException {
        switch (scoreState) {
            case LEFT_L2 -> {
                return RobotStates.LEFT_L2_FOLLOWTHROUGH;
            }
            case LEFT_L3 -> {
                return RobotStates.LEFT_L3_FOLLOWTHROUGH;
            }
            case LEFT_L4 -> {
                return RobotStates.LEFT_L4_FOLLOWTHROUGH;
            }
            default -> {
                throw new IllegalArgumentException("Please enter a valid Robot State");
            }
        }
    }

//    public Command openToScoreCommand(RobotStates scoreState) {
//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//                        setCurrentStateCommand(scoreState),
//                        new WaitUntilCommand(atPositionTrigger),
//                        setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE)),
//                handoffCommand(),
//                gripperSubsystem.hasCoralTrigger);
//    }

    //    public Command scoreCommand() {
//        return new ParallelCommandGroup(
//                gripperSubsystem.releaseCoral(),
//                setCurrentStateCommand(followThroughMap.get(currentState))
//        );
//    }
//
    public Command L1ScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(RobotStates.PRE_L1),
                new WaitCommand(0.3),
                setCurrentStateCommand(RobotStates.SCORE_L1),
                new RunCommand(() -> {
                }).until(() -> !intakeSubsystem.getBothSensorData().getAsBoolean()),
                new WaitCommand(0.2),
                setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
        );
    }

    public Command intakeCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(RobotStates.FLOOR_INTAKE),
                new RunCommand(() -> {
                }).until(intakeSubsystem.either),
                setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE));
    }

    public Command handoffCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(RobotStates.PRE_HANDOFF),
                new PrintCommand("1"),
                new WaitUntilCommand(atPositionTrigger),
                new PrintCommand("2"),
                setCurrentStateCommand(RobotStates.HANDOFF),
                new PrintCommand("3"),
                new WaitUntilCommand(readyToCloseTrigger),
                new PrintCommand("4"),
                setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
        );
    }


    public Command secureCommand() {
        return Commands.none();
    }

    public Command goToDefualtCommand() {
        return setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE);
    }

    @Log.NT
    public RobotStates getCurrentState() {
        return currentState;
    }

    @Log.NT
    public boolean getEither() {
        return intakeSubsystem.either.getAsBoolean();
    }

    @Log.NT
    public boolean getAtPostionTrigger() {
        return atPositionTrigger.getAsBoolean();
    }

}


