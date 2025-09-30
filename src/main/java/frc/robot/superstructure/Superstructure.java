package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.commands.CommandMutex;
import frc.robot.Constants;
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
import static frc.robot.superstructure.RobotState.LEFT_STAGE3_L2;
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

    private final HashMap<CoralScoreState, Command> scoreStage1CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage2CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage3CoralSideMap;
    private final HashMap<CoralScoreState, Command> scoreStage4CoralSideMap;

    private Superstructure.Process currentProcess;

    public RobotState currentState;
    public final Trigger atPositionTrigger;

    private final Trigger processChangeDefaultTrigger;
    private final Trigger processChangeIntakeCoralTrigger;
    private final Trigger processChangeIntakeAlgaeTrigger;
    private final Trigger processChangeScoreCoralTrigger;
    private final Trigger processChangeScoreAlgaeTrigger;

    private final LevelChangeTrigger levelChangeTrigger;

    private CommandMutex commandMutex;

    private Trigger isSwerveAtPlace;

    private Supplier<CoralScoreState> algaeHeightSuppier;

    public Superstructure(Trigger isSwerveAtPlace, Trigger alignmentTrigger) {
        currentState = DEFAULT_WITH_GAME_PIECE;

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
        processChangeScoreAlgaeTrigger = new Trigger(() -> currentProcess.equals(Process.SCORE_ALGAE));
        processChangeIntakeAlgaeTrigger = new Trigger(() -> currentProcess.equals(Process.INTAKE_ALGAE));
        processChangeScoreCoralTrigger = new Trigger(() -> currentProcess.equals(Process.SCORE_CORAL));
        processChangeIntakeCoralTrigger = new Trigger(() -> currentProcess.equals(Process.INTAKE_CORAL));

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

        commandMutex = new CommandMutex();

        processChangeDefaultTrigger.onTrue(commandMutex.scheduleCommand(defaultProcessCommand()));

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
                new SelectCommand<>(scoreStage1CoralSideMap, () -> coralScoreState),
                new WaitUntilCommand(atPositionTrigger),
                new PrintCommand("1"),
                new SelectCommand<>(scoreStage2CoralSideMap, () -> coralScoreState),
                new WaitUntilCommand(atPositionTrigger),
                new PrintCommand("2")
        );

        Command score = new SequentialCommandGroup(
                new SelectCommand<>(scoreStage3CoralSideMap, () -> coralScoreState),
                new WaitUntilCommand(atPositionTrigger),
                new PrintCommand("3"),
                new SelectCommand<>(scoreStage4CoralSideMap, () -> coralScoreState),
                new PrintCommand("#"),
                new WaitUntilCommand(atPositionTrigger),
                new PrintCommand("4")
        );

        return new SequentialCommandGroup(
                alignment.until(alignmentTrigger), // Todo
                score,
                this.setCurrentProcessCommand(Process.DEFAULT)
        );
    }

    public Command scoreAlgaeProcessCommand() {
        return new InstantCommand();
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
        return setCurrentStateCommand(DEFAULT_WITH_GAME_PIECE);
    }

    public Command handoffCommand() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(atPositionTrigger),
                setCurrentStateCommand(PRE_HANDOFF),
                new WaitUntilCommand(atPositionTrigger),
                new WaitCommand(HANDOFF_TIME_DELAY),
                setCurrentStateCommand(HANDOFF),
                new WaitUntilCommand(gripperSubsystem.hasGamePieceTrigger),
                new WaitCommand(HANDOFF_TIME_DELAY),
                setCurrentStateCommand(DEFAULT_WITH_GAME_PIECE)
        );
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
        return OpeningDirection.LEFT;
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
    public boolean getStagePosition() {
        return armSubsystem.getAngleSupplier() < LEFT_STAGE1_L2.armPosition.getAngle();
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
        SCORE_CORAL,
        SCORE_ALGAE,
        INTAKE_CORAL,
        INTAKE_ALGAE;
    }
}


