package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.control.motor.motor_specs.DirectionState;
import frc.excalib.mechanisms.Arm.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFXMotor m_angleMotor;
    private final CANcoder m_canCoder;
    private final DoubleSupplier m_angleSupplier;
    private ArmPosition m_currentState;
    private final Arm m_armMechanism;
    private final Trigger toleranceTrigger;
    private final ContinuousSoftLimit softLimit;
    private DoubleSupplier m_elevatorHeightSupplier;
    private BooleanSupplier m_intakeOpen;


    public ArmSubsystem() {
        m_currentState = ArmPosition.DEFAULT_WITHOUT_GAME_PIECE;
        m_angleMotor = new TalonFXMotor(ANGLE_MOTOR_ID);
        m_canCoder = new CANcoder(CAN_CODER_ID);
        m_angleSupplier = () -> m_canCoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;
        m_armMechanism = new Arm(m_angleMotor, m_angleSupplier, VELOCITY_LIMIT, new Gains(), new Mass(() -> 0, () -> 0, 0));
        toleranceTrigger = new Trigger(() -> (Math.abs(m_angleSupplier.getAsDouble() - m_currentState.getAngle()) < TOLERANCE)).debounce(0.1);
        m_elevatorHeightSupplier = () -> 0;
        m_intakeOpen = () -> false;
        m_angleMotor.setInverted(DirectionState.FORWARD);
        m_angleMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        m_angleMotor.setMotorPosition(POSITION_CONVERSION_FACTOR);

        softLimit = new ContinuousSoftLimit(
                () -> {
                    return m_elevatorHeightSupplier.getAsDouble() < ARM_LENGTH ? 0 : -Math.PI / 2;
                },
                () -> {
                    return m_elevatorHeightSupplier.getAsDouble() < ARM_LENGTH ? Math.PI : Math.PI * 1.5;
                }
        );

        setDefaultCommand(
                m_armMechanism.anglePositionControlCommand(
                        () -> softLimit.getSetpoint(m_angleSupplier.getAsDouble(), m_currentState.getAngle()),
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

    public Command coastCommand() {
        return new StartEndCommand(
                () -> m_angleMotor.setIdleState(COAST),
                () -> m_angleMotor.setIdleState(BRAKE),
                this
        ).ignoringDisable(true).withName("Arm Coast Command");
    }

    public DoubleSupplier getAngleSupplier() {
        return m_angleSupplier;
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        m_elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public void setIntakeOpen(BooleanSupplier intakeOpen) {
        m_intakeOpen = intakeOpen;
    }

    public BooleanSupplier isAtPosition() {
        return toleranceTrigger;
    }
}

