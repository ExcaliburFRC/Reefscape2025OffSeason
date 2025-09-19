package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.control.motor.motor_specs.IdleState;
import frc.excalib.mechanisms.linear_extension.LinearExtension;
import monologue.Annotations;
import monologue.Annotations.Log.NT;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.constants.*;

public class ElevatorSubsystem extends SubsystemBase implements Logged {
    private final TalonFXMotor leftMotor, rightMotor;
    private final MotorGroup motorGroup;

    private final DoubleSupplier elevatorHeight; //
    private final LinearExtension linearExtension;
    private ElevatorStates currentState; //
    private SoftLimit softLimit; //
    private DoubleSupplier armAngle; //
    public final Trigger atPositionTrigger; //
    public Trigger intakeOpenTrigger = new Trigger(() -> true);

    public ElevatorSubsystem() {
        rightMotor = new TalonFXMotor(RIGHT_MOTOR_ID);
        leftMotor = new TalonFXMotor(LEFT_MOTOR_ID);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        leftMotor.setInverted(DirectionState.FORWARD);
        rightMotor.setInverted(DirectionState.REVERSE);
        motorGroup = new MotorGroup(rightMotor, leftMotor);
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
        this.armAngle = () -> 0;

        setDefaultCommand(linearExtension.extendCommand(() -> softLimit.limit((currentState.getHeight())), this));

        atPositionTrigger = new Trigger(
                () -> (Math.abs(currentState.getHeight() - elevatorHeight.getAsDouble()) < TOLERANCE));

        softLimit = new SoftLimit(
                () -> {
                    if (isIntakeOpen()) {
                        return 0.4;
                    }
                    return 0;
                },
                () -> MAX_ELEVATOR_HIGHT
        );
    }

    public Command setStateCommand(ElevatorStates elevatorStates) {
        return new InstantCommand(() -> currentState = elevatorStates, this).withTimeout(0.05);
    }

    public Command manualCommand(DoubleSupplier voltage) {
        return linearExtension.manualCommand(voltage, this);
    }

    public void setIntakeOpenTrigger(Trigger triggerToSet) {
        intakeOpenTrigger = triggerToSet;
    }

    @NT
    public double getElevatorHeight() {
        return elevatorHeight.getAsDouble();
    }

    public void setArmAngle(DoubleSupplier setArmAngle) {
        armAngle = setArmAngle;
    }

    @NT
    public ElevatorStates getCurrentState() {
        return currentState;
    }

    @NT
    public SoftLimit getSoftLimit() {
        return softLimit;
    }

    @NT
    public double getArmAngle() {
        return armAngle.getAsDouble();
    }

    @NT
    public Trigger getAtPositionTrigger() {
        return atPositionTrigger;
    }

    @NT
    public double getLeftMotorPostion() {
        return leftMotor.getMotorPosition();
    }

    @NT
    public double getRightMotorPosition() {
        return rightMotor.getMotorPosition();
    }

    @NT
    public boolean isIntakeOpen() {
        return intakeOpenTrigger.getAsBoolean();
    }

//    public void setElevatorHeight(double hight) {
//
//    }

}

