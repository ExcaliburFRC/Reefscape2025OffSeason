package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.Constants.*;

public class Intake extends SubsystemBase implements Logged {
    // === Hardware ===
    public final TalonFXMotor armMotor;
    private final TalonFXMotor rollersMotor, centerlizerMotor;
    public final Arm arm;
    private final Mechanism centralizer, rollers;
    private final DigitalInput limitSwitch;

    // === Inputs and Triggers ===
    private final AnalogInput rightSensor, leftSensor;
    private Trigger rightSensorTrigger;
    private Trigger leftSensorTrigger;
    private final Trigger sensorTrigger;
    public final Trigger either, both;
    private final Trigger atPosition;
    private final Trigger intakeOpen;
    public final Trigger hasCoral;
    private final Trigger limitSwitchTrigger;


    // == States ==
    private IntakeState currentState;
    private IntakeState defaultState;

    // === Limits and Suppliers ==
    private final SoftLimit armSoftLimit;
    public DoubleSupplier angleSupplier;
    private final CurrentLimitsConfigs limitsConfigs;

    public Intake() {
        armMotor = new TalonFXMotor(ARM_MOTOR_ID);
        rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_ID);
        centerlizerMotor = new TalonFXMotor(CENTERLIZER_MOTOR_ID);

        centralizer = new Mechanism(centerlizerMotor);
        rollers = new Mechanism(rollersMotor);

        limitSwitch = new DigitalInput(9);

        armMotor.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        armMotor.setVelocityConversionFactor(ARM_POSITION_CONVERSION_FACTOR);

        armMotor.setMotorPosition(ARM_POSITION_CONVERSION_FACTOR);

        armMotor.setInverted(DirectionState.FORWARD);

        defaultState = IntakeState.DEFAULT;
        currentState = defaultState;

        limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 40;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        armMotor.getConfigurator().apply(limitsConfigs);
        rollersMotor.getConfigurator().apply(limitsConfigs);
        centerlizerMotor.getConfigurator().apply(limitsConfigs);

        angleSupplier = armMotor::getMotorPosition;

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        limitSwitchTrigger = new Trigger(() -> !limitSwitch.get());

        rightSensor = new AnalogInput(RIGHT_SENSOR_CHANNEL);
        leftSensor = new AnalogInput(LEFT_SENSOR_CHANNEL);

        sensorTrigger = new Trigger(
                () -> (getLeftSensorData() & !getRightSensorData()
                        || !getLeftSensorData() && getRightSensorData())
        );

        rightSensorTrigger = new Trigger(() -> false);
        leftSensorTrigger = new Trigger(() -> false);

        atPosition = new Trigger(
                () -> Math.abs(currentState.intakeAngle - angleSupplier.getAsDouble()) < TOLERANCE
        );

        sensorTrigger.onTrue(new PrintCommand("The Trigger Changed!"));

        intakeOpen = new Trigger(() -> angleSupplier.getAsDouble() < 1.2);

        arm = new Arm(
                armMotor,
                armMotor::getMotorPosition,
                new SoftLimit(
                        () -> ARM_VELOCITY_MIN,
                        () -> ARM_VELOCITY_MAX
                ),
                new Gains(3, 0, 0.25, 0, 0, 0, 0.9),
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble()),
                        () -> Math.sin(angleSupplier.getAsDouble()),
                        1
                )
        );

        armSoftLimit = new SoftLimit(
                () -> 0.8635,
                () -> 3.18
        );

        hasCoral = new Trigger(() -> (true)).debounce(0.1);

        limitSwitchTrigger.whileTrue(resetAngleCommand());

        setDefaultCommand(goToStateCommand());

        either = new Trigger(() -> (getRightSensorData() || getLeftSensorData()));
        both = new Trigger(() -> (getRightSensorData() && getLeftSensorData()));

        leftSensorTrigger = new Trigger(() -> rightSensor.getValue() < 4000);
        rightSensorTrigger = new Trigger(() -> leftSensor.getValue() < 4000);
    }


    public Command manualCommand(double armVoltage) {
        return arm.manualCommand(() -> armVoltage);
    }

    public Command goToStateCommand() {
        Command command = new ParallelCommandGroup(
                rollers.manualCommand(() -> currentState.rollerVoltage),
                centralizer.manualCommand(() -> currentState.centraliserVoltage),
                arm.anglePositionControlCommand(
                        () -> armSoftLimit.limit(currentState.intakeAngle),
                        at -> at = false,
                        TOLERANCE
                )
        );
        command.addRequirements(this);
        return command;
    }

    public Command setStateCommand(IntakeState state) {
        return new InstantCommand(() -> this.currentState = state);
    }

    public void returnToDefaultState() {
        this.currentState = this.defaultState;
    }

    public BooleanSupplier isIntakeOpen() {
        return intakeOpen;
    }

    public Trigger isAtPosition() {
        return atPosition;
    }

    @NT
    public boolean getLeftSensorData() {
        return !leftSensorTrigger.getAsBoolean();
    }

    @NT
    public boolean getRightSensorData() {
        return !rightSensorTrigger.getAsBoolean();
    }

    @NT
    public boolean getTriggerData() {
        return sensorTrigger.getAsBoolean();
    }

    @NT
    public BooleanSupplier getBothSensorData() {
        return () -> (getRightSensorData() && getLeftSensorData());
    }

    public Command intakeCommand() {
        Command command = new SequentialCommandGroup(
                setStateCommand(IntakeState.FLOOR_INTAKE),
                new WaitUntilCommand(either),
                setStateCommand(IntakeState.CENTERLIZE),
                new WaitUntilCommand(both),
                setStateCommand(IntakeState.DEFAULT)
        );
        command.addRequirements(this);
        return command;
    }

    public Command handoffCommand() {
        return setStateCommand(IntakeState.EJECT_CORAL);
    }

    @NT
    public double getAngleSupplier() {
        return angleSupplier.getAsDouble();
    }

    @NT
    public double getSetpoint() {
        return currentState.intakeAngle;
    }

    @NT
    public double getLimitedSetpoint() {
        return armSoftLimit.limit(currentState.intakeAngle);
    }

    @NT
    public String getCurrentState() {
        return currentState.name();
    }

    @NT
    public String getDefaultState() {
        return defaultState.name();
    }

    @NT
    public boolean getAtPosition() {
        return atPosition.getAsBoolean();
    }

    @NT
    public boolean getEither() {
        return either.getAsBoolean();
    }

    public Command resetAngleCommand() {
        return new RunCommand(() -> armMotor.setMotorPosition(ARM_DEFAULT_START_RAD)
        ).ignoringDisable(true);
    }


}
