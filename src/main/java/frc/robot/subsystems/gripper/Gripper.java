package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final TalonFXMotor gripperMotor;
    public final Trigger hasCoralTrigger; //
    public final Trigger hasAlgaeTrigger; //
    public final AnalogInput sensor = new AnalogInput(0);
    public GripperStates currentState = GripperStates.VACENT;

    // === Inputs ===
    private final FlyWheel gripperWheels;

    public Gripper() {
        gripperMotor = new TalonFXMotor(MOTOR_ID);

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 20;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        gripperMotor.getConfigurator().apply(limitsConfigs);
        gripperWheels = new FlyWheel(
                gripperMotor,
                MAX_ACCELERATION,
                MAX_JERK,
                new Gains()
        );
        hasCoralTrigger = new Trigger(() -> sensor.getValue() < 150).debounce(0.15);
        hasAlgaeTrigger = new Trigger(() -> HAS_CORAL_CURRENT < gripperWheels.logCurrent()).debounce(0.1);

        setDefaultCommand(gripperWheels.manualCommand(() -> currentState.output, this));
    }


    public Command setStateCommand(GripperStates stateToSet){
        return new InstantCommand(()-> currentState = stateToSet);
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
