package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.Constants.*;

public class Intake extends SubsystemBase {
    // === Motors ===
    private final TalonFXMotor m_armMotor;
    private final TalonFXMotor m_rollersMotor;

    // === Inputs ===
    private final DigitalInput m_sensor;
    private IntakeState m_currentState;
    private IntakeState m_defaultState;
    private final Mechanism m_centralizer, m_rollers;
    private final Arm m_arm;
    private final Trigger m_atPosition;
    private final CANcoder m_armEncoder;
    private final DoubleSupplier m_angleSupplier;
    private final Trigger intakeOpen;

    public Intake(IntakeState initialState) {

        m_currentState = initialState;
        m_defaultState = IntakeState.STOW;
        m_armMotor = new TalonFXMotor(ARM_MOTOR_ID);
        m_rollersMotor = new TalonFXMotor(ROLLERS_MOTOR_ID);
        TalonFXMotor m_centralizerMotor = new TalonFXMotor(CENTERLIZER_MOTOR_ID);

        m_sensor = new DigitalInput(SENSOR_CHANNEL);
        m_armEncoder = new CANcoder(ENCODER_ID);
        m_angleSupplier = () -> m_armEncoder.getPosition().getValueAsDouble();
        m_centralizer = new Mechanism(m_centralizerMotor);
        m_rollers = new Mechanism(m_rollersMotor);

        m_atPosition = new Trigger(
                () -> Math.abs(m_angleSupplier.getAsDouble() - m_currentState.intakeAngle) < TOLERANCE);

        intakeOpen = new Trigger(() -> (m_atPosition.getAsBoolean() && (m_currentState == IntakeState.FLOOR_INTAKE)));
        m_arm = new Arm(
                m_armMotor,
                m_angleSupplier,
                new SoftLimit(() -> ARM_VELOCITY_MIN, () -> ARM_VELOCITY_MAX),
                new Gains(),
                new Mass((() -> Math.cos(m_angleSupplier.getAsDouble())),
                        (() -> Math.sin(m_angleSupplier.getAsDouble())), 0)


        );
    }

    public Command defaultCommand() {
        return new ParallelCommandGroup(
                m_arm.anglePositionControlCommand(
                        () -> m_currentState.intakeAngle,
                        at -> at = m_atPosition.getAsBoolean(),
                        TOLERANCE,
                        this
                ),
                m_rollers.manualCommand(() -> m_currentState.rollerVoltage, this),
                m_centralizer.manualCommand(() -> m_currentState.centraliserVoltage, this)
        );
    }

    public void setState(IntakeState state) {
        this.m_currentState = state;
    }

    public void returnToDefaultState() {
        this.m_currentState = this.m_defaultState;
    }

    public BooleanSupplier isIntakeOpen(){
        return intakeOpen;
    }

    public BooleanSupplier isAtPosition(){
        return m_atPosition;
    }


}
