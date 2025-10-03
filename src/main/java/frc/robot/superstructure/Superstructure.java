package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.commands.CommandMutex;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.*;
import monologue.Logged;

import java.util.HashMap;
import java.util.function.Supplier;

import static frc.robot.Constants.SuperstructureConstants.HANDOFF_TIME_DELAY;
import static frc.robot.superstructure.RobotState.*;
import static frc.robot.superstructure.RobotState.SCORE_L2;
import static monologue.Annotations.*;

public class Superstructure implements Logged {
    // === Subsystems ==
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final Intake intakeSubsystem;
    public final Gripper gripperSubsystem;
//    public final ClimberSubsystem climberSubsystem;

    private CoralScoreState coralScoreState;
    private AlgaeScoreState algaeScoreState;

    private final HashMap<CoralScoreState, Command> preScoreCoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreCoralSideMap;
    private final HashMap<CoralScoreState, Command> postScoreCoralSideMap;

    private Superstructure.Process currentProcess;

    public RobotState currentState;
    public final Trigger atPositionTrigger;

    private final Trigger processChangeDefaultTrigger;
    private final Trigger processChangeCoralDefaultTrigger;
    private final Trigger processChangeAlgaeDefaultTrigger;
    private final Trigger processChangeIntakeCoralTrigger;
    private final Trigger processChangeIntakeAlgaeTrigger;
    private final Trigger processChangeScoreCoralTrigger;
    private final Trigger processChangeScoreAlgaeTrigger;

    private final LevelChangeTrigger levelChangeTrigger;

    private final CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;

    private Supplier<CoralScoreState> algaeHeightSuppier;

    public Superstructure(Trigger isSwerveAtPlace, Trigger alignmentTrigger) {
        currentState = DEFAULT_WITH_CORAL;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();
//        climberSubsystem = new ClimberSubsystem();

        currentProcess = Process.DEFAULT;
        algaeScoreState = AlgaeScoreState.NET;
        coralScoreState = CoralScoreState.L1;
        algaeHeightSuppier = () -> CoralScoreState.L2;

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean());

        levelChangeTrigger = new LevelChangeTrigger(() -> this.coralScoreState);

        processChangeDefaultTrigger = new Trigger(() -> currentProcess.equals(Process.DEFAULT));
        processChangeCoralDefaultTrigger = new Trigger(() -> currentProcess.equals(Process.CORAL_DEFAULT));
        processChangeAlgaeDefaultTrigger = new Trigger(() -> currentProcess.equals(Process.ALGAE_DEFAULT));
        processChangeScoreAlgaeTrigger = new Trigger(() -> currentProcess.equals(Process.SCORE_ALGAE));
        processChangeIntakeAlgaeTrigger = new Trigger(() -> currentProcess.equals(Process.INTAKE_ALGAE));
        processChangeScoreCoralTrigger = new Trigger(() -> currentProcess.equals(Process.SCORE_CORAL));
        processChangeIntakeCoralTrigger = new Trigger(() -> currentProcess.equals(Process.INTAKE_CORAL));

        preScoreCoralSideMap = new HashMap<>();
        scoreCoralSideMap = new HashMap<>();
        postScoreCoralSideMap = new HashMap<>();

        preScoreCoralSideMap.put(CoralScoreState.L1, reverseHandoffComamnd().andThen(getCoralStateProcessCommand(PRE_L1)));
        preScoreCoralSideMap.put(CoralScoreState.L2, handoffCommand().andThen(getCoralStateProcessCommand(PRE_L2)));
        preScoreCoralSideMap.put(CoralScoreState.L3, handoffCommand().andThen(getCoralStateProcessCommand(PRE_L3)));
        preScoreCoralSideMap.put(CoralScoreState.L4, handoffCommand().andThen(getCoralStateProcessCommand(PRE_L4)));

        scoreCoralSideMap.put(CoralScoreState.L1, getCoralStateProcessCommand(SCORE_L1));
        scoreCoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(SCORE_L2));
        scoreCoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(SCORE_L3));
        scoreCoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(SCORE_L4));

        postScoreCoralSideMap.put(CoralScoreState.L1, getCoralStateProcessCommand(DEFAULT_WITHOUT_GAME_PIECE));
        postScoreCoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(POST_L2));
        postScoreCoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(POST_L3));
        postScoreCoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(POST_L4));

        commandMutex = new CommandMutex();

        processChangeDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultProcessCommand()));

        processChangeAlgaeDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultAlageProcessCommand()));

        processChangeCoralDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultCoralProcessCommand()));

        processChangeIntakeCoralTrigger.onTrue(commandMutex.scheduleCommand(intakeCoralProcessCommand()));

        processChangeScoreCoralTrigger.onTrue(commandMutex.scheduleCommand(scoreCoralProcessCommand(alignmentTrigger)));

        processChangeScoreAlgaeTrigger.onTrue(commandMutex.scheduleCommand(scoreAlgaeProcessCommand()));

        processChangeIntakeAlgaeTrigger.onTrue(commandMutex.scheduleCommand(intakeAlgaeProcessCommand()));

        elevatorSubsystem.setArmAngleSuppier(armSubsystem::getAngleSupplier);
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(new Trigger(intakeSubsystem.isIntakeOpen()));
        elevatorSubsystem.setIntakeOpenTrigger(intakeSubsystem.isIntakeOpen());

        this.isSwerveAtPlace = isSwerveAtPlace;
    }

    public Command setCurrentStateCommand(RobotState state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentState = state),
                new PrintCommand("Changed State to:" + state),
                intakeSubsystem.setStateCommand(state.intakeState),
                armSubsystem.setStateCommand(state.armPosition),
                elevatorSubsystem.setStateCommand(state.elevatorState),
                gripperSubsystem.setStateCommand(state.gripperState)
        ).until(atPositionTrigger);
    }

    public Command scoreCoralProcessCommand(Trigger alignmentTrigger) {
        Command alignment = new SequentialCommandGroup(
                new SelectCommand<>(preScoreCoralSideMap, () -> coralScoreState)
        );

        Command score = new SequentialCommandGroup(
                new SelectCommand<>(scoreCoralSideMap, () -> coralScoreState),
                new SelectCommand<>(postScoreCoralSideMap, () -> coralScoreState)
        );

        return new ConditionalCommand(
                new InstantCommand(), // L1 todo
                new SequentialCommandGroup(
                        alignment.until(alignmentTrigger), // Todo
                        score,
                        this.setCurrentProcessCommand(Process.DEFAULT)
                ),
                () -> coralScoreState.equals(CoralScoreState.L1)
        );
    }

    public Command scoreAlgaeProcessCommand() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        proccesorScoreCommand(),
                        netScoreCommand(),
                        () -> algaeScoreState.equals(AlgaeScoreState.PROCESSOR)
                ),
                setCurrentProcessCommand(Process.DEFAULT)
        );
    }

    public Command intakeCoralProcessCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(FLOOR_INTAKE),
                new WaitUntilCommand(intakeSubsystem.either),
                setCurrentStateCommand(CENTERLIZE),
                new WaitUntilCommand(intakeSubsystem.both),
                new WaitCommand(0.2),
                setCurrentProcessCommand(Process.CORAL_DEFAULT)
        );
    }

    public Command intakeAlgaeProcessCommand() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        reverseHandoffComamnd(),
                        new InstantCommand(),
                        gripperSubsystem.hasGamePieceTrigger // by Coral
                ),

                getAlgaeIntakeCommand(AlgaeIntakeState.L2), // chnage to by by slice
                new WaitUntilCommand(() -> true), // algae present and robot is at safe distance from reef // todo
                setCurrentProcessCommand(Process.ALGAE_DEFAULT)

        );
    }

    public Command defaultProcessCommand() {
        return setCurrentStateCommand(DEFAULT_WITHOUT_GAME_PIECE);
    }

    public Command defaultCoralProcessCommand() {
        return new ConditionalCommand(
                setCurrentStateCommand(L1_DEFAULT),
                new SequentialCommandGroup(
                        handoffCommand(),
                        setCurrentStateCommand(DEFAULT_WITH_CORAL)
                ),
                () -> coralScoreState.equals(CoralScoreState.L1)
        );
    }

    public Command defaultAlageProcessCommand() {
        return setCurrentStateCommand(DEFAULT_WITH_ALGAE);
    }

    public Command handoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(PRE_HANDOFF),
                        new WaitCommand(HANDOFF_TIME_DELAY),
                        setCurrentStateCommand(HANDOFF),
                        new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger), // by coral
                        new WaitCommand(HANDOFF_TIME_DELAY)
                ),
                new PrintCommand("There is no coral in the intake!!!"),
                intakeSubsystem.either);
    }

    public Command secureCommand() {
        return Commands.none();  // Todo
    }

    @Log.NT
    public RobotState getCurrentState() {
        return currentState;
    }

    public Command getCoralStateProcessCommand(RobotState robotState) {
        return setCurrentStateCommand(robotState);
    }

    public OpeningDirection getScoringSide() {
        return OpeningDirection.LEFT; //todo
    }

    public Command getAlgaeIntakeCommand(AlgaeIntakeState state) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(ALGAE2_INTAKE)
                ),
                new SequentialCommandGroup(
                        setCurrentStateCommand(ALGAE3_INTAKE)
                ),
                () -> state.equals(AlgaeIntakeState.L2)
        );

//        return new ConditionalCommand(
//                new ConditionalCommand(
//                        setCurrentStateCommand(LEFT_ALGAE2),
//                        setCurrentStateCommand(RIGHT_ALGAE2),
//                        () -> getScoringSide().equals(OpeningDirection.LEFT)
//                ),
//                new ConditionalCommand(
//                        setCurrentStateCommand(LEFT_ALGAE3),
//                        setCurrentStateCommand(RIGHT_ALGAE3),
//                        () -> getScoringSide().equals(OpeningDirection.LEFT)
//                ),
//                () -> state.equals(AlgaeIntakeState.L2)
//        );
    }

    public Command reverseHandoffComamnd() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(PRE_REVERSE_HANDOFF),
                        new WaitCommand(HANDOFF_TIME_DELAY),
                        setCurrentStateCommand(REVERSE_HANDOFF),
                        new WaitUntilCommand(intakeSubsystem.either),
                        new WaitCommand(HANDOFF_TIME_DELAY)
                ),
                new PrintCommand("there is no coral is the gripper!!!!!!!!"),
                gripperSubsystem.setCoralStateTrigger);
    }

    public Command netScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(NET_SCORE_STAGE_1),
                setCurrentStateCommand(NET_SCORE_STAGE_2),
                setCurrentStateCommand(NET_SCORE_STAGE_3),
                new WaitUntilCommand(gripperSubsystem.hasAlgaeTrigger.negate().debounce(0.3)),
                setCurrentStateCommand(NET_SCORE_STAGE_4)
        );
    }

    public Command proccesorScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(NET_SCORE_STAGE_1),
                setCurrentStateCommand(NET_SCORE_STAGE_2),
                new WaitUntilCommand(gripperSubsystem.hasAlgaeTrigger.negate().debounce(0.3))
        );
    }

    public void setAlgaeScoreState(AlgaeScoreState algaeScoreState) {
        this.algaeScoreState = algaeScoreState;
    }

    public void setAlgaeHeightSuppier(CoralScoreState algaeHeightSuppier) {
        this.algaeHeightSuppier = () -> algaeHeightSuppier;
    }

    public Command setCurrentProcessCommand(Process currentProcess) {
        return new InstantCommand(() -> this.currentProcess = currentProcess);
    }

    public Command setCoralScoreStateCommand(CoralScoreState coralScoreState) {
        return new InstantCommand(() -> this.coralScoreState = coralScoreState);
    }

    @Log.NT
    public boolean getTriggerIntakeAlgae() {
        return processChangeIntakeAlgaeTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getTriggerScoreAlgae() {
        return processChangeScoreCoralTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getTriggerIntakeCoral() {
        return processChangeIntakeCoralTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getEither() {
        return intakeSubsystem.either.getAsBoolean();
    }

    @Log.NT
    public boolean getAtPostionTrigger() {
        return atPositionTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getTriggerScoreCoral() {
        return processChangeScoreCoralTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getDefaultTrigger() {
        return processChangeDefaultTrigger.getAsBoolean();
    }

    @Log.NT
    public Process getCurrentProcessSupplier() {
        return currentProcess;
    }

    @Log.NT
    public String getCoralScoreState() {
        return coralScoreState.name();
    }

    public enum Process {
        DEFAULT,
        CORAL_DEFAULT,
        ALGAE_DEFAULT,
        SCORE_CORAL,
        SCORE_ALGAE,
        INTAKE_CORAL,
        INTAKE_ALGAE;
    }

    private enum AlgaeIntakeState {
        L2, L3
    }
}


