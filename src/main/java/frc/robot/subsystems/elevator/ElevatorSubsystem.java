package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.constants.*;

public class ElevatorSubsystem extends SubsystemBase implements Logged {
    // === Hardware ===
    private final TalonFXMotor leftMotor, rightMotor;
    private final LinearExtension linearExtension;
    private final MotorGroup motorGroup;

    // === Suppliers and States ===
    private final DoubleSupplier elevatorHeight;
    private DoubleSupplier armAngleSuppier;
    private ElevatorStates currentState;

    // === Triggers ===
    public final Trigger atPositionTrigger;
    public Trigger intakeOpenTrigger;
    private SoftLimit softLimit;


    public ElevatorSubsystem() {
        rightMotor = new TalonFXMotor(RIGHT_MOTOR_ID);
        leftMotor = new TalonFXMotor(LEFT_MOTOR_ID);

        motorGroup = new MotorGroup(rightMotor, leftMotor);

        motorGroup.setMotorPosition(0);

        leftMotor.setInverted(DirectionState.FORWARD);
        rightMotor.setInverted(DirectionState.REVERSE);

        motorGroup.setIdleState(IdleState.BRAKE);

        motorGroup.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        motorGroup.setPositionConversionFactor(POSITION_CONVERSION_FACTOR);

        elevatorHeight = () -> (leftMotor.getMotorPosition() + rightMotor.getMotorPosition()) / 2;

        linearExtension = new LinearExtension(
                motorGroup,
                elevatorHeight,
                ELEVATOR_ANGLE,
                new Gains(0, 0, 0, 0, 7.78, 0, 0.15),
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),
                TOLERANCE
        );

        currentState = ElevatorStates.DEFAULT_WITH_GAME_PIECE;

        atPositionTrigger = new Trigger(
                () -> (Math.abs(currentState.getHeight() - elevatorHeight.getAsDouble()) < TOLERANCE)
        );

        armAngleSuppier = () -> 0;

        intakeOpenTrigger = new Trigger(() -> true);

        softLimit = new SoftLimit(
                () -> {
                    if (!isIntakeOpen()) {
                        return 0.32;
                    } else {
                        return 0;
                    }
                },
                () -> MAX_ELEVATOR_HIGHT
        );

        setDefaultCommand(
                linearExtension.extendCommand(
                        () -> softLimit.limit(currentState.getHeight()),
                        this
                )
        );
    }

    public Command manualCommand(DoubleSupplier voltage) {
        return linearExtension.manualCommand(voltage, this);
    }

    public Command setStateCommand(ElevatorStates elevatorStates) {
        return new InstantCommand(
                () -> currentState = elevatorStates
        );
    }

    public void setIntakeOpenTrigger(BooleanSupplier condition) {
        intakeOpenTrigger = new Trigger(condition);
    }

    @NT
    public double getElevatorHeight() {
        return elevatorHeight.getAsDouble();
    }

    public void setArmAngleSuppier(DoubleSupplier setArmAngle) {
        armAngleSuppier = setArmAngle;
    }

    @NT
    public double getSetpoint() {
        return currentState.getHeight();
    }

    @NT
    public SoftLimit getSoftLimit() {
        return softLimit;
    }

    @NT
    public boolean getAtPositionTrigger() {
        return atPositionTrigger.getAsBoolean();
    }

    @NT
    public boolean isIntakeOpen() {
        return intakeOpenTrigger.getAsBoolean();
    }

    public Command setElevatorHeightCommand(double hight) {
        return new InstantCommand(() -> motorGroup.setMotorPosition(0));
    }

    public Command zeroElevator() {
        return new InstantCommand(() -> motorGroup.setMotorPosition(0));
    }

    public Command coastCommand() {
        return new StartEndCommand(
                () -> motorGroup.setIdleState(IdleState.COAST),
                () -> motorGroup.setIdleState(IdleState.BRAKE)
        ).ignoringDisable(true);
    }

    @NT
    public double normilizeAngleSuppiler() {
        double temp = armAngleSuppier.getAsDouble() % (Math.PI * 2);
        if (temp < 0) {
            return temp += 2 * Math.PI;
        }
        return temp;
    }
}

