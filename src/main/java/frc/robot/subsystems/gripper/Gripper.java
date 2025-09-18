package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.math.EMAFilter;
import frc.excalib.control.math.periodics.PeriodicScheduler;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.fly_wheel.FlyWheel;
import monologue.Logged;

import static frc.robot.subsystems.gripper.GripperConstants.*;
import static monologue.Annotations.*;

public class Gripper extends SubsystemBase implements Logged {
    // === Motors ===
    private final TalonFXMotor m_rollersMotor;
    public final Trigger hasCoralTrigger; //
    public final Trigger hasAlgaeTrigger; //
    public final AnalogInput sensor = new AnalogInput(0);
    EMAFilter sensorFilter;

    // === Inputs ===
    private final FlyWheel m_gripperWheels;

    public Gripper() {
        m_rollersMotor = new TalonFXMotor(MOTOR_ID);

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 20;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        m_rollersMotor.getConfigurator().apply(limitsConfigs);
        m_gripperWheels = new FlyWheel(
                m_rollersMotor,
                MAX_ACCELERATION,
                MAX_JERK,
                new Gains()
        );
        hasCoralTrigger = new Trigger(() -> sensor.getValue() < 150);
        hasAlgaeTrigger = new Trigger(() -> HAS_CORAL_CURRENT < m_gripperWheels.logCurrent()).debounce(0.1);

        sensorFilter = new EMAFilter(sensor::getValue, 0.4, PeriodicScheduler.PERIOD.MILLISECONDS_10);
    }

    public Command releaseCoral() {
        return m_gripperWheels.manualCommand(() -> RELEASE_CORAL_VOLTAGE, this);

    }

    public Command intakeCoral() {
        return new RunCommand(() -> m_gripperWheels.setVoltage(INTAKE_CORAL_VOLTAGE), this);

    }

    public Command releaseAlgae() {
        return m_gripperWheels.manualCommand(() -> RELEASE_CORAL_VOLTAGE, this).until(hasAlgaeTrigger.negate());

    }

    public Command intakeAlgae() {
        return m_gripperWheels.manualCommand(() -> INTAKE_CORAL_VOLTAGE, this).until(hasAlgaeTrigger);

    }


    @Log.NT
    public boolean getHasAlgaeTrigger() {
        return hasAlgaeTrigger.getAsBoolean();
    }

    @Log.NT
    public boolean getHasCoralTrigger() {
        return hasCoralTrigger.getAsBoolean();
    }

    @Log.NT
    public double getSensorValue() {
        return sensor.getValue();
    }

}
