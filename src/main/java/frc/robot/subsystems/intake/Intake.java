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
    // === Motors ===
    public final TalonFXMotor armMotor, centerlizerMotor;
    private final TalonFXMotor rollersMotor;
    private final DigitalInput limitSwitch;
    private final Trigger limitSwitchTrigger;
    private final CurrentLimitsConfigs limitsConfigs;

    // === Inputs ===
    private final AnalogInput rightSensor, leftSensor; // . getVsl
    private Trigger rightSensorTrigger;
    private Trigger leftSensorTrigger;
    private final Trigger sensorTrigger;
    private final Trigger either, both;
    private IntakeState currentState; //
    private IntakeState defaultState; //
    private final Mechanism centralizer, rollers;
    public final Arm arm;
    private final Trigger atPosition; //

    private final SoftLimit armSoftLimit;

    public DoubleSupplier angleSupplier; //
    private final Trigger intakeOpen; //
    public final Trigger hasCoral;//

    public Intake(IntakeState initialState) {

        currentState = initialState;
        defaultState = IntakeState.DEFAULT;
        armMotor = new TalonFXMotor(ARM_MOTOR_ID);
        rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_ID);
        centerlizerMotor = new TalonFXMotor(CENTERLIZER_MOTOR_ID);
        armMotor.setPositionConversionFactor((Math.PI * 2 / 14.8));//move to a constant
        armMotor.setVelocityConversionFactor((Math.PI * 2 / 14.8));//same

        limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 40;
        limitsConfigs.SupplyCurrentLimitEnable = true;
        armMotor.getConfigurator().apply(limitsConfigs);
        rollersMotor.getConfigurator().apply(limitsConfigs);
        centerlizerMotor.getConfigurator().apply(limitsConfigs);

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        limitSwitch = new DigitalInput(9);
        limitSwitchTrigger = new Trigger(() -> !limitSwitch.get());

        armMotor.setMotorPosition(Math.PI / 2 - 0.7072895200359199);
        armMotor.setInverted(DirectionState.FORWARD);
        rightSensor = new AnalogInput(RIGHT_SENSOR_CHANNEL);
        leftSensor = new AnalogInput(LEFT_SENSOR_CHANNEL);

        sensorTrigger = new Trigger(() -> (getLeftSensorData() & !getRightSensorData() || !getLeftSensorData() && getRightSensorData()));

        rightSensorTrigger = new Trigger(() -> false);
        leftSensorTrigger = new Trigger(() -> false);

        angleSupplier = armMotor::getMotorPosition;
        centralizer = new Mechanism(centerlizerMotor);
        rollers = new Mechanism(rollersMotor);

        atPosition = new Trigger(() -> Math.abs(currentState.intakeAngle - angleSupplier.getAsDouble()) < TOLERANCE);
        sensorTrigger.onTrue(new PrintCommand("The Trigger Changed!"));

        intakeOpen = new Trigger(() -> (atPosition.getAsBoolean() && (currentState == IntakeState.FLOOR_INTAKE))).debounce(0.1);
        arm = new Arm(
                armMotor,
                armMotor::getMotorPosition,
                new SoftLimit(() -> ARM_VELOCITY_MIN, () -> ARM_VELOCITY_MAX),
                new Gains(3, 0, 0.25, 0, 0, 0, 0.9),
                new Mass((() -> Math.cos(angleSupplier.getAsDouble())), (() -> Math.sin(angleSupplier.getAsDouble())), 1));

        armSoftLimit = new SoftLimit(
                () -> {
                    return 0.8635;
                },
                () -> {
                    return 3.18;
                }
        );

        hasCoral = new Trigger(() -> (true)).debounce(0.1);

        limitSwitchTrigger.whileTrue(resetAngleCommand());

        setDefaultCommand(goToStateCommand());

        either = new Trigger(() -> (getRightSensorData() || getLeftSensorData()));
        both = new Trigger(() -> (getRightSensorData() && getLeftSensorData()));

        leftSensorTrigger = new Trigger(() -> rightSensor.getValue() < 4000);
        rightSensorTrigger = new Trigger(() -> leftSensor.getValue() < 4000);
    }


    public Command manualCommand(DoubleSupplier db) {
        return arm.manualCommand(db::getAsDouble);
    }

    public Command goToStateCommand() {
        Command command = new SequentialCommandGroup(
                new InstantCommand(() -> rollersMotor.setVoltage(currentState.rollerVoltage)),
                new InstantCommand(() -> centerlizerMotor.setVoltage(currentState.centraliserVoltage)),
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

    public BooleanSupplier isAtPosition() {
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
    public double getRightSensor() {
        return rightSensor.getValue();
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
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @NT
    public BooleanSupplier getEitherSensorData() {
        return () -> (getRightSensorData() || getLeftSensorData());
    }

    public Command resetAngleCommand() {
        return new RunCommand(() -> armMotor.setMotorPosition(Math.PI / 2 - 0.7072895)).ignoringDisable(true); //to a constant
    }


}
