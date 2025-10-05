package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Logged;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static monologue.Annotations.*;

public class Gripper extends SubsystemBase implements Logged {
    // === Motors ===
    private final TalonFXMotor gripperMotor;
    public final AnalogInput sensor = new AnalogInput(0);
    public GripperStates currentState = GripperStates.VACENT;
    public Trigger setAlgaeStateTrigger;
    public Trigger setCoralStateTrigger;
    public Trigger setEmptyTrigger;
    private HoldingState currentHoldingState = HoldingState.EMPTY;
    private final Trigger hasGamePieceTrigger;

    public final Trigger hasCoral;
    public final Trigger hasAlgae;

    // === Inputs ===
    private final FlyWheel gripperWheels;

    public Gripper() {
        gripperMotor = new TalonFXMotor(MOTOR_ID);

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 20;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        gripperMotor.getConfigurator().apply(limitsConfigs);
        gripperWheels = new FlyWheel(
                gripperMotor,
                MAX_ACCELERATION,
                MAX_JERK,
                new Gains()
        );

        setDefaultCommand(gripperWheels.manualCommand(() -> currentState.output, this));

        hasAlgae = new Trigger(() -> currentHoldingState.equals(HoldingState.ALGAE));
        hasCoral = new Trigger(() -> currentHoldingState.equals(HoldingState.CORAL));

        hasGamePieceTrigger = new Trigger(() -> sensor.getValue() < 150).debounce(0.15);

        setCoralStateTrigger = new Trigger(
                () -> currentHoldingState.equals(HoldingState.CORAL_EXPECTED))
                .and(hasGamePieceTrigger);

        setCoralStateTrigger.onTrue(new InstantCommand(() -> currentHoldingState = HoldingState.CORAL));

        setAlgaeStateTrigger = new Trigger(
                () -> currentHoldingState.equals(HoldingState.ALAGE_EXPECTED))
                .and(hasGamePieceTrigger);

        setAlgaeStateTrigger.onTrue(new InstantCommand(() -> currentHoldingState = HoldingState.ALGAE));

        setEmptyTrigger = new Trigger(
                () -> currentHoldingState.equals(HoldingState.CORAL))
                .or(() -> currentHoldingState.equals(HoldingState.ALGAE))
                .and(hasGamePieceTrigger.negate());

        setEmptyTrigger.onTrue(new InstantCommand(() -> currentHoldingState = HoldingState.EMPTY));
    }


    public Command setStateCommand(GripperStates stateToSet) {
        return new SequentialCommandGroup(
                new ConditionalCommand(
                        new InstantCommand(
                                () -> this.currentHoldingState = HoldingState.CORAL_EXPECTED),
                        new InstantCommand(),
                        () -> stateToSet.equals(GripperStates.INTAKE_CORAL)),
                new InstantCommand(() -> currentState = stateToSet));
    }

    @Log.NT
    public boolean getAlgaeTrigger() {
        return hasAlgae.getAsBoolean();
    }

    @Log.NT
    public boolean getCoralTrigger() {
        return hasCoral.getAsBoolean();
    }

    @Log.NT
    public HoldingState getCurrentHoldingState() {
        return currentHoldingState;
    }

    @Log.NT
    public boolean getHasGamePieceTrigger() {
        return hasGamePieceTrigger.getAsBoolean();
    }

    @Log.NT
    public double getSensorValue() {
        return sensor.getValue();
    }

    enum HoldingState {
        ALGAE,
        CORAL,
        EMPTY,
        CORAL_EXPECTED,
        ALAGE_EXPECTED,
    }

}
