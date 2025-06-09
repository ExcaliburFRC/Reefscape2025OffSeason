package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.linear_extension.LinearExtension;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.constants.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFXMotor m_leftMotor, m_rightMotor;
    private final MotorGroup m_elevatorMotors;
    private final DoubleSupplier m_elevatorHeight;
    private final LinearExtension m_linearExtension;
    private ElevatorStates m_currentState;
    private ContinuousSoftLimit softLimit;
    private DoubleSupplier armAngle;
    public final Trigger atPositionTrigger;

    public ElevatorSubsystem() {
        m_rightMotor = new TalonFXMotor(RIGHT_MOTOR_ID);
        m_leftMotor = new TalonFXMotor(LEFT_MOTOR_ID);
        m_elevatorMotors = new MotorGroup(m_rightMotor, m_leftMotor);
        this.armAngle = () -> 0;
        m_elevatorHeight = m_elevatorMotors::getMotorPosition;
        m_linearExtension = new LinearExtension(
                m_elevatorMotors,
                m_elevatorHeight,
                ELEVATOR_ANGLE,
                new Gains(),
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION),
                TOLERANCE
        );
        m_currentState = ElevatorStates.DEFAULT;

        setDefaultCommand(m_linearExtension.extendCommand(() -> m_currentState.getHeight(), this));

        atPositionTrigger = new Trigger(()-> (Math.abs(m_currentState.getHeight() - m_elevatorHeight.getAsDouble()) < TOLERANCE));
        softLimit = new ContinuousSoftLimit(
                () -> MAX_ELEVATOR_HIGHT,
                () -> (armAngle.getAsDouble() > UPWARDS_ARM_MAX_LIMIT &&
                        armAngle.getAsDouble() < UPWARDS_ARM_MIN_LIMIT) ?
                        MIN_ELEVATOR_HIGHT : MIN_ELEVATOR_HIGHT_WITH_OPEN_ARM
        );
    }

    public void setState(ElevatorStates elevatorStates) {
        m_currentState = elevatorStates;
    }
    //

    public Command manualCommand(DoubleSupplier voltage) {
        return m_linearExtension.manualCommand(voltage, this);
    }

    public DoubleSupplier getElevatorHeight() {
        return m_elevatorHeight;
    }

    public void setArmAngle(DoubleSupplier setArmAngle) {
        armAngle = setArmAngle;
    }
}

