package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;

import static frc.robot.subsystems.gripper.GripperConstants.*;

public class Gripper extends SubsystemBase {
    // === Motors ===
    private final TalonFXMotor m_rollersMotor;
    private final Trigger m_hasCoralTrigger;
    // === Inputs ===
    private final FlyWheel m_gripperWheels;

    public Gripper() {
        m_rollersMotor = new TalonFXMotor(MOTOR_ID);
        m_gripperWheels = new FlyWheel(
                m_rollersMotor,
                MAX_ACCELIRATION,
                MAX_JERK,
                new Gains()
        );
        m_hasCoralTrigger = new Trigger(() -> HAS_CORAL_CURRENT < m_gripperWheels.logCurrent());
    }

    public Command releaseCoral() {
        return m_gripperWheels.manualCommand(() -> RELEASE_CORAL_VOLTAGE, this).until(m_hasCoralTrigger.negate());

    }

    public Command intakeCoral() {
        return m_gripperWheels.manualCommand(() -> INTAKE_CORAL_VOLTAGE, this).until(m_hasCoralTrigger);

    }

}
