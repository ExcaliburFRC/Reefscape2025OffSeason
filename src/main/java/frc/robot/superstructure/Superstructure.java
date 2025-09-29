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

import static frc.robot.superstructure.RobotState.*;
import static frc.robot.superstructure.RobotState.LEFT_STAGE3_L2;
import static monologue.Annotations.*;

public class Superstructure implements Logged {
    // === Subsystems ==
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final Intake intakeSubsystem;
    public final Gripper gripperSubsystem;
    public final ClimberSubsystem climberSubsystem;

    private CoralScoreState coralScoreState;
    private AlgaeScoreState algaeScoreState;

    private final HashMap<CoralScoreState, Command> scoreStage1CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage2CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage3CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage4CoralSideMap;

    private Superstructure.Process currentProcess;

    private final HashMap<Superstructure.Process, Command> processCommandHashMap;
    public RobotState currentState;
    public final Trigger atPositionTrigger;

    private final ProcessChangeTrigger processChangeTrigger;
    private final LevelChangeTrigger levelChangeTrigger;

    private CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;

    private Supplier<CoralScoreState> algaeHeightSuppier;


    public Superstructure(Trigger isSwerveAtPlace) {
        currentState = DEFAULT_WITHOUT_GAME_PIECE;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();
        climberSubsystem = new ClimberSubsystem();

        currentProcess = Process.DEFAULT;
        algaeScoreState = AlgaeScoreState.NET;
        coralScoreState = CoralScoreState.L1;
        algaeHeightSuppier = () -> CoralScoreState.L2;

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean()
        );

        levelChangeTrigger = new LevelChangeTrigger(() -> this.coralScoreState);

        processChangeTrigger = new ProcessChangeTrigger(() -> currentProcess);

        scoreStage1CoralSideMap = new HashMap<>();
        scoreStage2CoralSideMap = new HashMap<>();
        scoreStage3CoralSideMap = new HashMap<>();
        scoreStage4CoralSideMap = new HashMap<>();

        scoreStage1CoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(LEFT_STAGE1_L2));
        scoreStage1CoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(LEFT_STAGE1_L3));
        scoreStage1CoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(LEFT_STAGE1_L4));

        scoreStage2CoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(LEFT_STAGE2_L2));
        scoreStage2CoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(LEFT_STAGE2_L3));
        scoreStage2CoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(LEFT_STAGE2_L4));

        scoreStage3CoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(LEFT_STAGE3_L2));
        scoreStage3CoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(LEFT_STAGE3_L3));
        scoreStage3CoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(LEFT_STAGE3_L4));

        scoreStage4CoralSideMap.put(CoralScoreState.L2, getCoralStateProcessCommand(LEFT_STAGE4_L2));
        scoreStage4CoralSideMap.put(CoralScoreState.L3, getCoralStateProcessCommand(LEFT_STAGE4_L3));
        scoreStage4CoralSideMap.put(CoralScoreState.L4, getCoralStateProcessCommand(LEFT_STAGE4_L4));

        processCommandHashMap = new HashMap<>();
        processCommandHashMap.put(Process.DEFAULT, defaultProcessCommand());
        processCommandHashMap.put(Process.SCORE_ALGAE, new InstantCommand());
        processCommandHashMap.put(Process.SCORE_CORAL, scoreCoralProcessCommand());
        processCommandHashMap.put(Process.INTAKE_ALGAE, new InstantCommand());
        processCommandHashMap.put(Process.INTAKE_CORAL, intakeCoralProcessCommand());

        commandMutex = new CommandMutex();

        processChangeTrigger.getTrigger().onFalse(
                commandMutex.scheduleCommand(
                        new SelectCommand<Process>(
                                processCommandHashMap,
                                () -> currentProcess
                        )
                )
        );

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

    public Command scoreCoralProcessCommand() {
        Command alignment = new SequentialCommandGroup(
                new SelectCommand<>(scoreStage1CoralSideMap, () -> coralScoreState),
                new SelectCommand<>(scoreStage2CoralSideMap, () -> coralScoreState)
        );

        Command score = new SequentialCommandGroup(
                new SelectCommand<>(scoreStage3CoralSideMap, () -> coralScoreState),
                new SelectCommand<>(scoreStage4CoralSideMap, () -> coralScoreState)
        );


        return new SequentialCommandGroup(
                alignment.until(() -> new WaitCommand(5).isFinished()), //todo
                score,
                this.setCurrentProcessCommand(Process.DEFAULT)
        );
    }

    public Command scoreAlgaeProcessCommand() {
        return Commands.none(); //todo
//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//                        new WaitUntilCommand(isSwerveAtPlace),//positioned
//                        setCurrentStateCommand(null),//stage 1
//                        setCurrentStateCommand(null),//stage 2
//                        setCurrentStateCommand(null),//stage 3
//                        new WaitUntilCommand(null),//gripper empty
//                        setCurrentStateCommand(null)//stage 4
//                ),
//                new SequentialCommandGroup(
//                        new WaitUntilCommand(null),//positioned
//                        setCurrentStateCommand(null),//stage 1
//                        setCurrentStateCommand(null),//stage 2
//                        new WaitUntilCommand(null)//empty gripper
//                ),
//                () -> algaeScoreState.equals(AlgaeScoreState.NET)
//        ).andThen(setCurrentProcessCommand(Process.DEFAULT));
    }

    public Command intakeCoralProcessCommand() {
        return new SequentialCommandGroup(
                setCurrentStateCommand(FLOOR_INTAKE),
                new PrintCommand("1"),
                new WaitUntilCommand(intakeSubsystem.either),
                new PrintCommand("2"),
                setCurrentStateCommand(DEFAULT_WITH_GAME_PIECE),
                new PrintCommand("3"),
                new WaitUntilCommand(intakeSubsystem.both),
                new WaitCommand(0.2),
                new PrintCommand("4"),
                handoffCommand(),
                new PrintCommand("5"),
                setCurrentProcessCommand(Process.DEFAULT)
        );
    }

    public Command intakeAlgaeProcessCommand() {


        Command emptyGripperCommand = new SequentialCommandGroup(
                setCurrentStateCommand(PRE_HANDOFF),
                setCurrentStateCommand(REVERSE_HANDOFF),
                new WaitUntilCommand(intakeSubsystem.either)
        );

        return new SequentialCommandGroup(
                new ConditionalCommand(
                        emptyGripperCommand,
                        new InstantCommand(),
                        gripperSubsystem.hasGamePieceTrigger
                ),
                getAlgaeScoreCommand(algaeHeightSuppier.get()),
                new WaitUntilCommand(() -> true), //algae present and robot is at safe distance from reef //todo
                setCurrentProcessCommand(Process.DEFAULT)

        );
    }

    public Command defaultProcessCommand() {
        return setCurrentStateCommand(DEFAULT_WITHOUT_GAME_PIECE);
    }

    public Command handoffCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(atPositionTrigger),
                setCurrentStateCommand(PRE_HANDOFF),
                new WaitUntilCommand(atPositionTrigger),
                new WaitCommand(0.2),
                setCurrentStateCommand(HANDOFF),
                new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger),
                new WaitCommand(0.2),
                setCurrentStateCommand(DEFAULT_WITH_GAME_PIECE)
        );
    }

    public Command secureCommand() {
        return Commands.none(); //todo
    }

    @Log.NT
    public RobotState getCurrentState() {
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

    @Log.NT
    public boolean getStagePosition() {
        return armSubsystem.getAngleSupplier() < LEFT_STAGE1_L2.armPosition.getAngle();
    }

    public Command setCoralScoreStateCommand(CoralScoreState coralScoreState) {
        return new InstantCommand(() -> this.coralScoreState = coralScoreState);
    }

    public void setAlgaeScoreState(AlgaeScoreState algaeScoreState) {
        this.algaeScoreState = algaeScoreState;
    }

    public OpeningDirection getScoringSide() {
        return OpeningDirection.LEFT;
    }

    public Command setCurrentProcessCommand(Process currentProcess) {
        return new InstantCommand(() -> this.currentProcess = currentProcess);
    }

    @Log.NT
    public Process getCurrentProcessSupplier() {
        return currentProcess;
    }

    @Log.NT
    public boolean getProcessChangeTrigger() {
        return processChangeTrigger.getTrigger().debounce(1.5).getAsBoolean();
    }

    @Log.NT
    public String getCoralScoreState() {
        return coralScoreState.name();
    }

    public Command getCoralStateProcessCommand(RobotState robotState) {
        return setCurrentStateCommand(robotState);
    }

    public void setAlgaeHeightSuppier(CoralScoreState algaeHeightSuppier) {
        this.algaeHeightSuppier = () -> algaeHeightSuppier;
    }

    public Command getAlgaeScoreCommand(CoralScoreState height) {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new ConditionalCommand(
                                setCurrentStateCommand(LEFT_ALGAE2),
                                setCurrentStateCommand(RIGHT_ALGAE2),
                                () -> getScoringSide().equals(OpeningDirection.LEFT)
                        ),
                        new ConditionalCommand(
                                setCurrentStateCommand(LEFT_ALGAE3),
                                setCurrentStateCommand(RIGHT_ALGAE3),
                                () -> getScoringSide().equals(OpeningDirection.LEFT)
                        ),
                        () -> height.equals(CoralScoreState.L2)
                )
        );
    }

    public enum Process {
        DEFAULT,
        SCORE_CORAL,
        SCORE_ALGAE,
        INTAKE_CORAL,
        INTAKE_ALGAE;
    }
}


