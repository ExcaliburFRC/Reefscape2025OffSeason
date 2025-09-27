package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.additional_utilities.DoubleKeyMap;
import frc.excalib.commands.CommandMutex;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AlgaeScoreState;
import frc.robot.util.CoralScoreState;
import frc.robot.util.LevelChangeTrigger;
import frc.robot.util.ProcessChangeTrigger;
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

    private CoralScoreState coralScoreState;
    private AlgaeScoreState algaeScoreState;

    private final DoubleKeyMap<CoralScoreState, ScoreSide, RobotStates> scoreStage1CoralSideMap;
    private final DoubleKeyMap<CoralScoreState, ScoreSide, RobotStates> scoreStage2CoralSideMap;
    private final DoubleKeyMap<CoralScoreState, ScoreSide, RobotStates> scoreStage3CoralSideMap;
    private final DoubleKeyMap<CoralScoreState, ScoreSide, RobotStates> scoreStage4CoralSideMap;

    private Superstructure.Process currentProcess;

    private final HashMap<Superstructure.Process, Command> processCommandHashMap;
    public RobotStates currentState;
    public final Trigger atPositionTrigger;
    public final Trigger readyToCloseTrigger;

    private final ProcessChangeTrigger processChangeTrigger;
    private final LevelChangeTrigger levelChangeTrigger;

    private CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;


    public Superstructure(Trigger isSwerveAtPlace) {
        currentState = RobotStates.DEFAULT_WITHOUT_GAME_PIECE;
        coralScoreState = CoralScoreState.L2;

        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        intakeSubsystem = new Intake();
        gripperSubsystem = new Gripper();
        climberSubsystem = new ClimberSubsystem();


        currentProcess = Process.DEFAULT;

        levelChangeTrigger = new LevelChangeTrigger(() -> this.coralScoreState);

        processChangeTrigger = new ProcessChangeTrigger(() -> currentProcess);

        scoreStage1CoralSideMap = new DoubleKeyMap<>();
        scoreStage2CoralSideMap = new DoubleKeyMap<>();
        scoreStage3CoralSideMap = new DoubleKeyMap<>();
        scoreStage4CoralSideMap = new DoubleKeyMap<>();

        scoreStage1CoralSideMap.put(CoralScoreState.L2, ScoreSide.LEFT, RobotStates.LEFT_STAGE1_L2);
        scoreStage1CoralSideMap.put(CoralScoreState.L3, ScoreSide.LEFT, RobotStates.LEFT_STAGE1_L3);
        scoreStage1CoralSideMap.put(CoralScoreState.L4, ScoreSide.LEFT, RobotStates.LEFT_STAGE1_L4);

        scoreStage1CoralSideMap.put(CoralScoreState.L2, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE1_L2);
        scoreStage1CoralSideMap.put(CoralScoreState.L3, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE1_L3);
        scoreStage1CoralSideMap.put(CoralScoreState.L4, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE1_L4);


        scoreStage2CoralSideMap.put(CoralScoreState.L2, ScoreSide.LEFT, RobotStates.LEFT_STAGE2_L2);
        scoreStage2CoralSideMap.put(CoralScoreState.L3, ScoreSide.LEFT, RobotStates.LEFT_STAGE2_L3);
        scoreStage2CoralSideMap.put(CoralScoreState.L4, ScoreSide.LEFT, RobotStates.LEFT_STAGE2_L4);

        scoreStage2CoralSideMap.put(CoralScoreState.L2, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE2_L2);
        scoreStage2CoralSideMap.put(CoralScoreState.L3, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE2_L3);
        scoreStage2CoralSideMap.put(CoralScoreState.L4, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE2_L4);


        scoreStage3CoralSideMap.put(CoralScoreState.L2, ScoreSide.LEFT, RobotStates.LEFT_STAGE3_L2);
        scoreStage3CoralSideMap.put(CoralScoreState.L3, ScoreSide.LEFT, RobotStates.LEFT_STAGE3_L3);
        scoreStage3CoralSideMap.put(CoralScoreState.L4, ScoreSide.LEFT, RobotStates.LEFT_STAGE3_L4);

        scoreStage3CoralSideMap.put(CoralScoreState.L2, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE3_L2);
        scoreStage3CoralSideMap.put(CoralScoreState.L3, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE3_L3);
        scoreStage3CoralSideMap.put(CoralScoreState.L4, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE3_L4);


        scoreStage4CoralSideMap.put(CoralScoreState.L2, ScoreSide.LEFT, RobotStates.LEFT_STAGE4_L2);
        scoreStage4CoralSideMap.put(CoralScoreState.L3, ScoreSide.LEFT, RobotStates.LEFT_STAGE4_L3);
        scoreStage4CoralSideMap.put(CoralScoreState.L4, ScoreSide.LEFT, RobotStates.LEFT_STAGE4_L4);

        scoreStage4CoralSideMap.put(CoralScoreState.L2, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE4_L2);
        scoreStage4CoralSideMap.put(CoralScoreState.L3, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE4_L3);
        scoreStage4CoralSideMap.put(CoralScoreState.L4, ScoreSide.RIGHT, RobotStates.RIGHT_STAGE4_L4);


        processCommandHashMap = new HashMap<>();
        processCommandHashMap.put(Process.DEFAULT, defaultProcessCommand());
        processCommandHashMap.put(Process.SCORE_ALGAE, scoreAlgaeProcessCommand());
        processCommandHashMap.put(Process.SCORE_CORAL, scoreCoralProcessCommand());
        processCommandHashMap.put(Process.INTAKE_ALGAE, intakeAlgaeProcessCommand());
        processCommandHashMap.put(Process.INTAKE_CORAL, intakeCoralProcessCommand());

        commandMutex = new CommandMutex();

        processChangeTrigger.onTrue(
                commandMutex.scheduleCommand(
                        new SelectCommand<Process>(
                                processCommandHashMap,
                                () -> currentProcess
                        )
                )
        );

        atPositionTrigger = new Trigger(
                () -> elevatorSubsystem.atPositionTrigger.getAsBoolean() &&
                        armSubsystem.isAtPosition() &&
                        intakeSubsystem.isAtPosition().getAsBoolean()
        );


        elevatorSubsystem.setArmAngleSuppier(armSubsystem::getAngleSupplier);
        armSubsystem.setElevatorHeightSupplier(elevatorSubsystem::getElevatorHeight);

        armSubsystem.setIntakeOpen(new Trigger(intakeSubsystem.isIntakeOpen()));
        elevatorSubsystem.setIntakeOpenTrigger(intakeSubsystem.isIntakeOpen());

        readyToCloseTrigger = new Trigger(atPositionTrigger.and(intakeSubsystem.hasCoral));

        this.isSwerveAtPlace = isSwerveAtPlace;
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

    public Command setCurrentProcessCommand(Process process) {
        return new InstantCommand(() -> this.currentProcess = process);
    }

    public Command scoreCoralProcessCommand() {
        Command alignment = new InstantCommand();

        Command score = new SequentialCommandGroup(
                setCurrentStateCommand(scoreStage3CoralSideMap.get(coralScoreState, getScoringSide())),
                setCurrentStateCommand(scoreStage4CoralSideMap.get(coralScoreState, getScoringSide()))
        );

        return new SequentialCommandGroup(
                alignment.until(() -> true),
                new ConditionalCommand(
                        score,
                        setCurrentStateCommand(scoreStage2CoralSideMap.get(coralScoreState, getScoringSide())), () -> true
                ),
                this.setCurrentProcessCommand(Process.DEFAULT)
        );
    }

    public Command scoreAlgaeProcessCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new WaitUntilCommand(null),//positioned
                        setCurrentStateCommand(null),//stage 1
                        setCurrentStateCommand(null),//stage 2
                        setCurrentStateCommand(null),//stage 3
                        new WaitUntilCommand(null),//gripper empty
                        setCurrentStateCommand(null)//stage 4
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(null),//positioned
                        setCurrentStateCommand(null),//stage 1
                        setCurrentStateCommand(null),//stage 2
                        new WaitUntilCommand(null)//empty gripper
                ),
                () -> algaeScoreState.equals(AlgaeScoreState.NET)
        ).andThen(setCurrentProcessCommand(Process.DEFAULT));
    }

    public Command intakeCoralProcessCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.FLOOR_INTAKE_WITH_ALGAE),
                        new WaitUntilCommand(intakeSubsystem.either),
                        setCurrentStateCommand(null),// closed intake
                        setCurrentProcessCommand(Process.DEFAULT)
                ),
                new SequentialCommandGroup(
                        setCurrentStateCommand(RobotStates.FLOOR_INTAKE),
                        new WaitUntilCommand(intakeSubsystem.either),
                        setCurrentStateCommand(null),// closed intake
                        setCurrentProcessCommand(Process.SCORE_CORAL)
                ),
                () -> true //algae present

        );
    }

    public Command intakeAlgaeProcessCommand() {
        Command emptyGripper = new SequentialCommandGroup(
                setCurrentStateCommand(null),//gripper above intake, intake rotating inwards
                setCurrentStateCommand(null),//passing the coral
                new WaitUntilCommand(intakeSubsystem.either)
        );
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        emptyGripper,
                        new InstantCommand(),
                        () -> true//coral in gripper
                ),
                setCurrentStateCommand(null),// algae intake state
                new WaitUntilCommand(() -> true),//algae present and robot is at safe distance from rif
                setCurrentProcessCommand(Process.DEFAULT)

        );
    }

    public Command defaultProcessCommand() {
        return new ConditionalCommand(
                setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE),
                setCurrentStateCommand(RobotStates.DEFAULT_WITHOUT_GAME_PIECE),
                () -> true
        );
    }


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
                new WaitCommand(0.2),
                new PrintCommand("2"),
                setCurrentStateCommand(RobotStates.HANDOFF),
                new PrintCommand("3"),
                new WaitUntilCommand(readyToCloseTrigger),
                new WaitCommand(0.2),
                new PrintCommand("4"),
                setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
        );
    }

    public Command returnToDefaultCommand() {
        return new RunCommand(() -> setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE));
    }

    public Command L2ScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        setCurrentStateCommand(RobotStates.LEFT_STAGE1_L2),
                                        new PrintCommand("1"),
                                        new WaitUntilCommand(atPositionTrigger),
                                        new PrintCommand("2")),
                                Commands.none(),
                                () -> (
                                        armSubsystem.getAngleSupplier() > RobotStates.LEFT_STAGE1_L2.armPosition.getAngle() &&
                                                armSubsystem.getAngleSupplier() < RobotStates.LEFT_STAGE2_L2.armPosition.getAngle())
                        ),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE2_L2),
                        new PrintCommand("3"),
                        new WaitUntilCommand(atPositionTrigger),
                        new PrintCommand("4"),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE3_L2),
                        new PrintCommand("5"),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE4_L2),
                        new PrintCommand("6"),
                        new WaitCommand(0.2),
                        new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
                ),
                new PrintCommand("there is no coral in the system"),
                gripperSubsystem.hasGamePieceTrigger
        );
    }

    public Command L3ScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        setCurrentStateCommand(RobotStates.LEFT_STAGE1_L3),
                                        new PrintCommand("1"),
                                        new WaitUntilCommand(atPositionTrigger),
                                        new PrintCommand("2")),
                                Commands.none(),
                                () -> (
                                        armSubsystem.getAngleSupplier() > RobotStates.LEFT_STAGE1_L3.armPosition.getAngle() &&
                                                armSubsystem.getAngleSupplier() < RobotStates.LEFT_STAGE3_L3.armPosition.getAngle())
                        ),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE2_L3),
                        new PrintCommand("3"),
                        new WaitUntilCommand(atPositionTrigger),
                        new PrintCommand("4"),
                        new WaitCommand(0.2),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE3_L3),
                        new PrintCommand("5"),
                        new WaitCommand(0.5),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE4_L3),
                        new PrintCommand("6"),
                        new WaitCommand(0.5),
                        new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
                ),
                new PrintCommand("there is no coral in the system"),
                gripperSubsystem.hasGamePieceTrigger
        );
    }

    public Command L4ScoreCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        setCurrentStateCommand(RobotStates.LEFT_STAGE1_L4),
                                        new PrintCommand("1"),
                                        new WaitUntilCommand(atPositionTrigger),
                                        new PrintCommand("2")),
                                Commands.none(),
                                () -> (
                                        armSubsystem.getAngleSupplier() > RobotStates.LEFT_STAGE1_L4.armPosition.getAngle() &&
                                                armSubsystem.getAngleSupplier() < RobotStates.LEFT_STAGE3_L4.armPosition.getAngle())
                        ),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE2_L4),
                        new PrintCommand("3"),
                        new WaitUntilCommand(atPositionTrigger),
                        new PrintCommand("4"),
                        new WaitCommand(0.2),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE3_L4),
                        new PrintCommand("5"),
                        new WaitCommand(0.5),
                        setCurrentStateCommand(RobotStates.LEFT_STAGE4_L4),
                        new PrintCommand("6"),
                        new WaitCommand(0.5),
                        new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger.negate()),
                        setCurrentStateCommand(RobotStates.DEFAULT_WITH_GAME_PIECE)
                ),
                new PrintCommand("there is no coral in the system"),
                gripperSubsystem.hasGamePieceTrigger
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

    @Log.NT
    public boolean getStagePosition() {
        return armSubsystem.getAngleSupplier() < RobotStates.LEFT_STAGE1_L2.armPosition.getAngle();
    }

    public Command setCoralScoreStateCommand(CoralScoreState coralScoreState) {
        return new InstantCommand(() -> this.coralScoreState = coralScoreState);
    }

    public void setAlgaeScoreState(AlgaeScoreState algaeScoreState) {
        this.algaeScoreState = algaeScoreState;
    }

    public enum Process {
        DEFAULT,
        SCORE_CORAL,
        SCORE_ALGAE,
        INTAKE_CORAL,
        INTAKE_ALGAE;
    }

    public enum ScoreSide {
        LEFT,
        RIGHT;
    }

    public ScoreSide getScoringSide() {
        return ScoreSide.LEFT;
    }


}


