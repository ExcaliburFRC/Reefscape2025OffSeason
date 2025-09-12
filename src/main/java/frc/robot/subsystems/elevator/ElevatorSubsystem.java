package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public ElevatorSubsystem() {
        rightMotor = new TalonFXMotor(RIGHT_MOTOR_ID);
        leftMotor = new TalonFXMotor(LEFT_MOTOR_ID);

        leftMotor.setInverted(DirectionState.REVERSE);
        rightMotor.setInverted(DirectionState.FORWARD);
        motorGroup = new MotorGroup(rightMotor, leftMotor);
        motorGroup.setIdleState(IdleState.BRAKE);
        motorGroup.setVelocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
        motorGroup.setPositionConversionFactor(VELOCITY_CONVERSION_FACTOR);


        elevatorHeight = leftMotor::getMotorPosition;

        linearExtension = new LinearExtension(
                motorGroup,
                elevatorHeight,
                ELEVATOR_ANGLE,
                new Gains(),
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),
                TOLERANCE
        );

        currentState = ElevatorStates.DEFAULT_WITH_GAME_PIECE;
        this.armAngle = () -> 0;

//        setDefaultCommand(linearExtension.extendCommand(() -> softLimit.limit(currentState.getHeight()), this));

        atPositionTrigger = new Trigger(
                () -> (Math.abs(currentState.getHeight() - elevatorHeight.getAsDouble()) < TOLERANCE));

        softLimit = new SoftLimit(
                () -> MIN_ELEVATOR_HIGHT,
                () -> MAX_ELEVATOR_HIGHT
        );
    }

    public void setState(ElevatorStates elevatorStates) {
        currentState = elevatorStates;
    }

    public Command manualCommand(DoubleSupplier voltage) {
        return linearExtension.manualCommand(voltage, this);
    }

    @Annotations.Log.NT
    public DoubleSupplier getElevatorHeight() {
        return elevatorHeight;
    }

    public void setArmAngle(DoubleSupplier setArmAngle) {
        armAngle = setArmAngle;
    }

    @Annotations.Log.NT
    public ElevatorStates getCurrentState() {
        return currentState;
    }

    @Annotations.Log.NT
    public SoftLimit getSoftLimit() {
        return softLimit;
    }

    @Annotations.Log.NT
    public DoubleSupplier getArmAngle() {
        return armAngle;
    }

    @Annotations.Log.NT
    public Trigger getAtPositionTrigger() {
        return atPositionTrigger;
    }
}

