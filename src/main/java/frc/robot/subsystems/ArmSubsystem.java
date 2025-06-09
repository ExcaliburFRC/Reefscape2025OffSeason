package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFXMotor m_angleMotor;
    private final CANcoder m_canCoder;
    private final DoubleSupplier m_angleSupplier;
    private ArmPosition m_currentState;
    private final Arm m_armMechanism;
    private final Trigger toleranceTrigger;
    private final ContinuousSoftLimit softLimit;
    private final DoubleSupplier m_elevatorHeightSupplier;
    private final BooleanSupplier m_intakeOpen;

    public ArmSubsystem(DoubleSupplier elevatorHeightSupplier, BooleanSupplier intakeOpen) {
        m_currentState = ArmPosition.DEFAULT;
        m_angleMotor = new TalonFXMotor(ANGLE_MOTOR_ID);
        m_canCoder = new CANcoder(CAN_CODER_ID);
        m_angleSupplier = () -> m_canCoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;
        m_armMechanism = new Arm(m_angleMotor, m_angleSupplier, VELOCITY_LIMIT, new Gains(), new Mass(() -> 0, () -> 0, 0));
        toleranceTrigger = new Trigger(() -> (Math.abs(m_angleSupplier.getAsDouble() - m_currentState.getAngle()) < TOLERANCE));
        m_elevatorHeightSupplier = elevatorHeightSupplier;
        m_intakeOpen = intakeOpen;

        softLimit = new ContinuousSoftLimit(
                () -> {
                    return (-1 * (Math.acos(m_elevatorHeightSupplier.getAsDouble() / ARM_LENGTH)) - SOFTLIMIT_BUFFER);
                },
                () -> {
                    return (Math.acos(m_elevatorHeightSupplier.getAsDouble() / ARM_LENGTH) + SOFTLIMIT_BUFFER);
                }
        );

        setDefaultCommand(
                m_armMechanism.anglePositionControlCommand(
                        () -> m_currentState.getAngle(),
                        (__) -> toleranceTrigger.getAsBoolean(),
                        TOLERANCE,
                        this
                )
        );
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return m_armMechanism.manualCommand(voltageSupplier, this);
    }

    public void setState(ArmPosition state) {
        m_currentState = state;
    }


}

