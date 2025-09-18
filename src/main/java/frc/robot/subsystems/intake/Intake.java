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
    private final DigitalInput digitalInput;

    // === Inputs ===
    private final AnalogInput rightSensor, leftSensor; // . getVsl
    private final Trigger rightSensorTrigger, leftSensorTrigger, sensor;
    private final Trigger either, both;
    private IntakeState currentState; //
    private IntakeState defaultState; //
    private final Mechanism centralizer, rollers;
    public final Arm arm;
    private final Trigger atPosition; //

    public final DoubleSupplier m_angleSupplier; //
    private final Trigger intakeOpen; //
    public final Trigger hasCoral;//

    public final SoftLimit positionLimit;

    public Intake(IntakeState initialState) {

        currentState = initialState;
        defaultState = IntakeState.DEFAULT;
        armMotor = new TalonFXMotor(ARM_MOTOR_ID);
        rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_ID);
        centerlizerMotor = new TalonFXMotor(CENTERLIZER_MOTOR_ID);
        armMotor.setPositionConversionFactor((Math.PI * 2 / 14.8));
        armMotor.setVelocityConversionFactor((Math.PI * 2 / 14.8));

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 50;
        limitsConfigs.SupplyCurrentLimitEnable = true;
        armMotor.getConfigurator().apply(limitsConfigs);

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        digitalInput = new DigitalInput(8);

        armMotor.setMotorPosition(Math.PI / 2 - 0.7072895200359199);
        armMotor.setInverted(DirectionState.FORWARD);
        rightSensor = new AnalogInput(RIGHT_SENSOR_CHANNEL);
        leftSensor = new AnalogInput(LEFT_SENSOR_CHANNEL);

        sensor = new Trigger(() -> (getLeftSensorData() & !getRightSensorData() || !getLeftSensorData() && getRightSensorData()));

        rightSensorTrigger = new Trigger(() -> leftSensor.getValue() < 4000);
        leftSensorTrigger = new Trigger(() -> rightSensor.getValue() < 4000);

        m_angleSupplier = armMotor::getMotorPosition;
        centralizer = new Mechanism(centerlizerMotor);
        rollers = new Mechanism(rollersMotor);

        atPosition = new Trigger(() -> Math.abs(currentState.intakeAngle - m_angleSupplier.getAsDouble()) < TOLERANCE);

        intakeOpen = new Trigger(() -> (atPosition.getAsBoolean() && (currentState == IntakeState.FLOOR_INTAKE))).debounce(0.1);
        arm = new Arm(
                armMotor,
                armMotor::getMotorPosition,
                new SoftLimit(() -> ARM_VELOCITY_MIN, () -> ARM_VELOCITY_MAX),
                new Gains(2.5, 0, 0, 0, 0, 0, 0.8),
                new Mass((() -> Math.cos(m_angleSupplier.getAsDouble())), (() -> Math.sin(m_angleSupplier.getAsDouble())), 1));

        hasCoral = new Trigger(() -> (true)).debounce(0.1);

        positionLimit = new SoftLimit(() -> 0, () -> 0);
        setDefaultCommand(goToStateCommand());
        either = new Trigger(() -> (getRightSensorData() || getLeftSensorData()));
        both = new Trigger(() -> (getRightSensorData() && getLeftSensorData()));


    }


    public Command manualCommand(DoubleSupplier db) {
        return arm.manualCommand(db::getAsDouble);
    }

    public Command goToStateCommand() {
        Command command = new ParallelCommandGroup(
                arm.anglePositionControlCommand(
                        () -> currentState.intakeAngle,
                        at -> at = false,
                        TOLERANCE).until(atPosition),
                new InstantCommand(() -> rollersMotor.setVoltage(currentState.rollerVoltage)),
                new InstantCommand(() -> centerlizerMotor.setVoltage(currentState.centraliserVoltage)));
        command.addRequirements(this);
        return command;

    }

    public Command setStateCommand(IntakeState state) {
        return new InstantCommand(() -> this.currentState = state, this);
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
        return sensor.getAsBoolean();
    }

    @NT
    public BooleanSupplier getBothSensorData() {
        return () -> (getRightSensorData() && getLeftSensorData());
    }

    public Command intakeCommand() {
        Command command = new SequentialCommandGroup(
                new PrintCommand("help"),
                setStateCommand(IntakeState.FLOOR_INTAKE),
                new PrintCommand("please work"),
                new WaitUntilCommand(either),
                new PrintCommand("it worked!"),
                setStateCommand(IntakeState.CENTERLIZE),
                new PrintCommand("2"),
                new WaitUntilCommand(both),
                new PrintCommand("3"),
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
        return m_angleSupplier.getAsDouble();
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
    public String getCurrentState() {
        return currentState.name();
    }

    @NT
    public String getDefaultState() {
        return defaultState.name();
    }

    @NT
    public boolean getDigitalInput() {
        return digitalInput.get();
    }

    @NT
    public BooleanSupplier getEitherSensorData() {
        return () -> (getRightSensorData() || getLeftSensorData());
    }
    @NT
    public BooleanSupplier getEither() {
        return either;
    }
    @NT
    public BooleanSupplier getBoth() {
        return both;
    }

}
