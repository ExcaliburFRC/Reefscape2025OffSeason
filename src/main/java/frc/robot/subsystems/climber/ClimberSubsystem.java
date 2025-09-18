package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.climber.Constants.*;

public class ClimberSubsystem extends SubsystemBase implements Logged {
    private final TalonFXMotor firstMotor, secondMotor;
    private final MotorGroup motorGroup;
    private final Arm climberMechanism;
    private final Trigger atPosition; //
    private DoubleSupplier setpoint; //

    public ClimberSubsystem() {
        firstMotor = new TalonFXMotor(MOTOR1_ID);
        secondMotor = new TalonFXMotor(MOTOR2_ID);

        motorGroup = new MotorGroup(firstMotor, secondMotor);

        motorGroup.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        motorGroup.setVelocityConversionFactor(ARM_POSITION_CONVERSION_FACTOR);

        motorGroup.setMotorPosition(INITIAL_START_ANGLE);

        climberMechanism = new Arm(motorGroup, motorGroup::getMotorPosition, VELOCITY_SOFTLIMIT, GAINS, MASS);

        setpoint = () -> INITIAL_START_ANGLE;

        atPosition = new Trigger(() -> ((setpoint.getAsDouble() - motorGroup.getMotorPosition()) < TOLERANCE));
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return climberMechanism.manualCommand(voltageSupplier, this);
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = () -> setpoint;
    }

    public Command open() {
        return Commands.none();
    }

    public Command retract() {
        return Commands.none();
    }

     @Log.NT
    public DoubleSupplier getSetpoint() {
        return setpoint;
    }

    @Log.NT
    public Trigger getAtPosition() {
        return atPosition;
    }
}
