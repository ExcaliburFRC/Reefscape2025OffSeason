package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFXMotor angleMotor;
    private final CANcoder canCoder;
    private final DoubleSupplier angleSupplier;
    private ArmPosition m_currentState;
    private final Arm armMechanism;
    private final Trigger toleranceTrigger;

    public ArmSubsystem() {
        m_currentState = ArmPosition.DEFAULT;
        angleMotor = new TalonFXMotor(ANGLE_MOTOR_ID);
        canCoder = new CANcoder(CAN_CODER_ID);
        angleSupplier = () -> canCoder.getPosition().getValueAsDouble() * 360;
        armMechanism = new Arm(angleMotor, angleSupplier, VELOCITY_LIMIT, new Gains(), new Mass(() -> 0, () -> 0, 0));
        toleranceTrigger = new Trigger(() -> (Math.abs(angleSupplier.getAsDouble() - m_currentState.getAngle()) < TOLERANCE));

        setDefaultCommand(
                armMechanism.anglePositionControlCommand(
                        () -> m_currentState.getAngle(),
                        at -> at = toleranceTrigger.getAsBoolean(),
                        TOLERANCE,
                        this
                )
        );
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public void setState(ArmPosition state) {
        m_currentState = state;
    }


}

