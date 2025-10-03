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

    private final Trigger intakeAlgaeTrigger;
    private final Trigger intakeCoralTrigger;
    private final Trigger scoreAlgaeTrigger;
    private final Trigger scoreCoralTrigger;

    private final Trigger hasCoralInRobot;

    private final LevelChangeTrigger levelChangeTrigger;

    private final CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;

    private Supplier<CoralScoreState> algaeHeightSuppier;

    public Superstructure(Trigger isSwerveAtPlace, Trigger alageButton, Trigger coralButton) {
        currentState = DEFAULT_WITH_CORAL;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();

        currentProcess = Process.DEFAULT;
        algaeScoreState = AlgaeScoreState.NET;
        coralScoreState = CoralScoreState.L1;
        algaeHeightSuppier = () -> CoralScoreState.L2;

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean());

        levelChangeTrigger = new LevelChangeTrigger(() -> this.coralScoreState);
        hasCoralInRobot = gripperSubsystem.hasCoral.or(intakeSubsystem.hasCoral);


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

        processChangeScoreCoralTrigger.onTrue(commandMutex.scheduleCommand(scoreCoralProcessCommand(isSwerveAtPlace)));

        processChangeScoreAlgaeTrigger.onTrue(commandMutex.scheduleCommand(scoreAlgaeProcessCommand()));

        processChangeIntakeAlgaeTrigger.onTrue(commandMutex.scheduleCommand(intakeAlgaeProcessCommand()));

        elevatorSubsystem.setArmAngleSuppier(armSubsystem::getAngleSupplier);
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(new Trigger(intakeSubsystem.isIntakeOpen()));
        elevatorSubsystem.setIntakeOpenTrigger(intakeSubsystem.isIntakeOpen());


        intakeAlgaeTrigger = alageButton
                .and(gripperSubsystem.hasAlgae.negate())
                .and(() -> currentProcess.equals(Process.CORAL_DEFAULT) || currentProcess.equals(Process.DEFAULT));

        intakeCoralTrigger = coralButton
                .and(hasCoralInRobot.negate())
                .and(() -> currentProcess.equals(Process.ALGAE_DEFAULT) || currentProcess.equals(Process.DEFAULT));

        scoreAlgaeTrigger = alageButton.and(gripperSubsystem.hasAlgae)
                .and(() -> currentProcess.equals(Process.ALGAE_DEFAULT));

        scoreCoralTrigger = coralButton
                .and(gripperSubsystem.hasAlgae.negate())
                .and(gripperSubsystem.hasCoral)
                .and(() -> currentProcess.equals(Process.CORAL_DEFAULT));

        intakeAlgaeTrigger.onTrue(setCurrentProcessCommand(Process.INTAKE_ALGAE));
        intakeCoralTrigger.onTrue(setCurrentProcessCommand(Process.INTAKE_CORAL));

        scoreCoralTrigger.onTrue(setCurrentProcessCommand(Process.SCORE_CORAL));
        scoreAlgaeTrigger.onTrue(setCurrentProcessCommand(Process.SCORE_ALGAE));

        this.isSwerveAtPlace = isSwerveAtPlace;
    }

    private Command setCurrentStateCommand(RobotState state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentState = state),
                new PrintCommand("Changed State to:" + state),
                intakeSubsystem.setStateCommand(state.intakeState),
                armSubsystem.setStateCommand(state.armPosition),
                elevatorSubsystem.setStateCommand(state.elevatorState),
                gripperSubsystem.setStateCommand(state.gripperState)
        ).until(atPositionTrigger);
    }

    private Command scoreCoralProcessCommand(Trigger alignmentTrigger) {
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

    private Command scoreAlgaeProcessCommand() {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        proccesorScoreCommand(),
                        netScoreCommand(),
                        () -> algaeScoreState.equals(AlgaeScoreState.PROCESSOR)
                ),
                new ConditionalCommand(
                        setCurrentProcessCommand(Process.CORAL_DEFAULT),
                        setCurrentProcessCommand(Process.DEFAULT),
                        hasCoralInRobot
                )
        );
    }

    private Command intakeCoralProcessCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(FLOOR_INTAKE),
                new WaitUntilCommand(intakeSubsystem.either),
                setCurrentStateCommand(CENTERLIZE),
                new WaitUntilCommand(intakeSubsystem.both),
                new WaitCommand(0.2),
                setCurrentProcessCommand(Process.CORAL_DEFAULT)
        );
    }

    private Command intakeAlgaeProcessCommand() {
        return new SequentialCommandGroup(
                reverseHandoffComamnd(),
                getAlgaeIntakeCommand(AlgaeIntakeState.L2), // chnage to by by slice
                new WaitUntilCommand(() -> true), // algae present and robot is at safe distance from reef // todo
                setCurrentProcessCommand(Process.ALGAE_DEFAULT)

        );
    }

    private Command defaultProcessCommand() {
        return setCurrentStateCommand(DEFAULT_WITHOUT_GAME_PIECE);
    }

    private Command defaultCoralProcessCommand() {
        return new ConditionalCommand(
                setCurrentStateCommand(L1_DEFAULT),
                new SequentialCommandGroup(
                        handoffCommand(),
                        setCurrentStateCommand(DEFAULT_WITH_CORAL)
                ),
                () -> coralScoreState.equals(CoralScoreState.L1)
        );
    }

    private Command defaultAlageProcessCommand() {
        return setCurrentStateCommand(DEFAULT_WITH_ALGAE);
    }

    private Command handoffCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(PRE_HANDOFF),
                        new WaitCommand(HANDOFF_TIME_DELAY),
                        setCurrentStateCommand(HANDOFF),
                        new WaitUntilCommand(gripperSubsystem.hasCoral), // by coral
                        new WaitCommand(HANDOFF_TIME_DELAY)
                ),
                new PrintCommand("There is no coral in the intake!!!"),
                intakeSubsystem.either);
    }

    private Command secureCommand() {
        return Commands.none();  // Todo
    }

    @Log.NT
    public RobotState getCurrentState() {
        return currentState;
    }

    public Command getCoralStateProcessCommand(RobotState robotState) {
        return setCurrentStateCommand(robotState);
    }

    @Log.NT
    private OpeningDirection getScoringSide() {
        return OpeningDirection.LEFT; //todo
    }

    private Command getAlgaeIntakeCommand(AlgaeIntakeState state) {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(ALGAE2_INTAKE)
                ),
                new SequentialCommandGroup(
                        setCurrentStateCommand(ALGAE3_INTAKE)
                ),
                () -> state.equals(AlgaeIntakeState.L2)
        );
    }

    private Command reverseHandoffComamnd() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(PRE_REVERSE_HANDOFF),
                        new WaitCommand(HANDOFF_TIME_DELAY),
                        setCurrentStateCommand(REVERSE_HANDOFF),
                        new WaitUntilCommand(intakeSubsystem.either),
                        new WaitCommand(HANDOFF_TIME_DELAY)
                ),
                new PrintCommand("there is no coral is the gripper!!!!!!!!"),
                gripperSubsystem.hasCoral);
    }

    private Command netScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(NET_SCORE_STAGE_1),
                setCurrentStateCommand(NET_SCORE_STAGE_2),
                setCurrentStateCommand(NET_SCORE_STAGE_3),
                new WaitUntilCommand(gripperSubsystem.hasAlgae.negate().debounce(0.3)),
                setCurrentStateCommand(NET_SCORE_STAGE_4)
        );
    }

    private Command proccesorScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(NET_SCORE_STAGE_1),
                setCurrentStateCommand(NET_SCORE_STAGE_2),
                new WaitUntilCommand(gripperSubsystem.hasAlgae.negate().debounce(0.3))
        );
    }

    public Command setAlgaeScoreStateCommand(AlgaeScoreState algaeScoreState) {
        return new InstantCommand(() -> this.algaeScoreState = algaeScoreState);
    }

    private Command setCurrentProcessCommand(Process currentProcess) {
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


