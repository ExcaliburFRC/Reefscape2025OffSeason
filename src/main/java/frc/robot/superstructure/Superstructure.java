package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.commands.CommandMutex;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
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

    public final Trigger coralButton;

    private final Trigger l2Slice;
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

    private final Trigger safeCloseTrigger;

    private final Trigger hasCoralInRobot;

    private final LevelChangeTrigger levelChangeTrigger;

    private final CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;

    private Supplier<CoralScoreState> algaeHeightSuppier;

    public Superstructure(Trigger isSwerveAtPlace, Trigger alageButton, Trigger coralButton, Trigger safeCloseTrigger, Trigger l2Slice, Trigger leftRiffScoreTrigger, Trigger cancelTrigger) {
        currentState = DEFAULT_WITH_CORAL;

        armSubsystem = new ArmSubsystem(leftRiffScoreTrigger);
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();

        this.coralButton = coralButton;
        this.safeCloseTrigger = safeCloseTrigger;

        currentProcess = Process.CORAL_DEFAULT;
        algaeScoreState = AlgaeScoreState.NET;
        coralScoreState = CoralScoreState.L1;
        algaeHeightSuppier = () -> CoralScoreState.L2;

        this.l2Slice = l2Slice;

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean()
        );

        cancelTrigger.onTrue(
                new ConditionalCommand(
                        setCurrentProcessCommand(Process.ALGAE_DEFAULT),
                        new ConditionalCommand(
                                setCurrentProcessCommand(Process.CORAL_DEFAULT),
                                setCurrentProcessCommand(Process.DEFAULT),
                                this.gripperSubsystem.hasCoral.or(this.intakeSubsystem.either)
                        ),
                        gripperSubsystem.hasAlgae
                )
        );

        levelChangeTrigger = new LevelChangeTrigger(() -> this.coralScoreState);
        hasCoralInRobot = gripperSubsystem.hasCoral.or(intakeSubsystem.either);

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

        preScoreCoralSideMap.put(CoralScoreState.L1, reverseHandoffComamnd().andThen(setCurrentStateCommand(PRE_L1), new WaitCommand(0.1)));
        preScoreCoralSideMap.put(CoralScoreState.L2, handoffCommand().andThen(setCurrentStateCommand(PRE_L2)));
        preScoreCoralSideMap.put(CoralScoreState.L3, handoffCommand().andThen(setCurrentStateCommand(PRE_L3)));
        preScoreCoralSideMap.put(CoralScoreState.L4, handoffCommand().andThen(setCurrentStateCommand(PRE_L4)));

        scoreCoralSideMap.put(CoralScoreState.L1, setCurrentStateCommand(SCORE_L1).andThen(new WaitUntilCommand(intakeSubsystem.either.negate().debounce(0.4))));
        scoreCoralSideMap.put(CoralScoreState.L2, setCurrentStateCommand(SCORE_L2));
        scoreCoralSideMap.put(CoralScoreState.L3, setCurrentStateCommand(SCORE_L3_1).andThen(setCurrentStateCommand(SCORE_L3_2)));
        scoreCoralSideMap.put(CoralScoreState.L4, setCurrentStateCommand(SCORE_L4_1).andThen(setCurrentStateCommand(SCORE_L4_2)));

        postScoreCoralSideMap.put(CoralScoreState.L1, setCurrentStateCommand(DEFAULT_WITHOUT_GAME_PIECE));
        postScoreCoralSideMap.put(CoralScoreState.L2, setCurrentStateCommand(POST_L2));
        postScoreCoralSideMap.put(CoralScoreState.L3, new WaitCommand(0.3).andThen(setCurrentStateCommand(POST_L3)));
        postScoreCoralSideMap.put(CoralScoreState.L4, new WaitCommand(0.24).andThen(setCurrentStateCommand(POST_L4)));

        commandMutex = new CommandMutex();

        processChangeDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultProcessCommand()));

        processChangeAlgaeDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultAlageProcessCommand()));

        processChangeCoralDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultCoralProcessCommand()));

        processChangeIntakeCoralTrigger.onTrue(commandMutex.scheduleCommand(intakeCoralProcessCommand()));

        processChangeScoreCoralTrigger.onTrue(commandMutex.scheduleCommand(scoreCoralProcessCommand()));

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
                .and(hasCoralInRobot)
                .and(() -> currentProcess.equals(Process.CORAL_DEFAULT));

        intakeAlgaeTrigger.onTrue(setCurrentProcessCommand(Process.INTAKE_ALGAE));
        intakeCoralTrigger.onTrue(setCurrentProcessCommand(Process.INTAKE_CORAL));

        scoreCoralTrigger.onTrue(setCurrentProcessCommand(Process.SCORE_CORAL));
        scoreAlgaeTrigger.onTrue(setCurrentProcessCommand(Process.SCORE_ALGAE));

        this.isSwerveAtPlace = isSwerveAtPlace;
    }

    public Command setCurrentStateCommand(RobotState state) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> currentState = state),
                new PrintCommand("Changed State to:" + state),
                intakeSubsystem.setStateCommand(state.intakeState),
                armSubsystem.setStateCommand(state.armPosition),
                elevatorSubsystem.setStateCommand(state.elevatorState),
                gripperSubsystem.setStateCommand(state.gripperState),
                new WaitUntilCommand(atPositionTrigger).ignoringDisable(true)
        ).ignoringDisable(true);
    }

    private Command scoreCoralProcessCommand() {
        Command alignment = new SequentialCommandGroup(
                new SelectCommand<>(preScoreCoralSideMap, () -> coralScoreState)
        );

        Command score = new SequentialCommandGroup(
                new SelectCommand<>(scoreCoralSideMap, () -> coralScoreState),
                new SelectCommand<>(postScoreCoralSideMap, () -> coralScoreState)
        );

        return new SequentialCommandGroup(
                alignment,
                new WaitUntilCommand(coralButton),
                score,
                new WaitUntilCommand(safeCloseTrigger),
                this.setCurrentProcessCommand(Process.DEFAULT)
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
                setCurrentStateCommand(FLOOR_INTAKE).until(intakeSubsystem.either),
                new WaitUntilCommand(intakeSubsystem.either),
                setCurrentStateCommand(CENTERLIZE),
                new WaitUntilCommand(intakeSubsystem.both),
                new WaitCommand(0.2),
                setCurrentProcessCommand(Process.CORAL_DEFAULT)
        ).ignoringDisable(true);
    }

    private Command intakeAlgaeProcessCommand() {
        return new SequentialCommandGroup(
                reverseHandoffComamnd(),
                new ConditionalCommand(
                        getAlgaeIntakeCommand(AlgaeIntakeState.L2),
                        getAlgaeIntakeCommand(AlgaeIntakeState.L3),
                        this.l2Slice),
                new WaitUntilCommand(gripperSubsystem.hasAlgae),// algae present and robot is at safe distance from reef // todo
                new WaitUntilCommand(safeCloseTrigger),
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
                        handoffCommand().until(gripperSubsystem.hasCoral),
                        setCurrentStateCommand(DEFAULT_WITH_CORAL)
                ),
                () -> coralScoreState.equals(CoralScoreState.L1)
        );
    }

    public Command ejectCommand() {
        return setCurrentStateCommand(RobotState.EJECT)
                .withTimeout(1)
                .andThen(defaultProcessCommand());
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
                        new WaitUntilCommand(gripperSubsystem.hasCoral) // by coral
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
                gripperSubsystem.hasCoral
        );
    }

    private Command netScoreCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(NET_SCORE_STAGE_1),
                setCurrentStateCommand(NET_SCORE_STAGE_2),
                setCurrentStateCommand(NET_SCORE_STAGE_3),
                new WaitUntilCommand(gripperSubsystem.hasAlgae.negate().debounce(0.3)),
                setCurrentStateCommand(NET_SCORE_STAGE_4),
                setCurrentStateCommand(NET_SCORE_STAGE_5)
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
        return new InstantCommand(() -> this.currentProcess = currentProcess).ignoringDisable(true);
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
        return processChangeScoreAlgaeTrigger.getAsBoolean();
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

    @Log.NT
    public boolean getSafeCloseTrigger() {
        return safeCloseTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getCoralButton() {
        return coralButton.getAsBoolean();
    }
}


