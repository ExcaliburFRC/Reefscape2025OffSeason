package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.climber.ClimberConstants.*;

public class Climber extends SubsystemBase {
    private final TalonFXMotor motor1, motor2;
    private final MotorGroup motorGroup;
    private final Arm climberMechanism;
    private final Trigger atPosition;
    private DoubleSupplier kSetpoint;

    public Climber() {
        motor1 = new TalonFXMotor(MOTOR1_ID);
        motor2 = new TalonFXMotor(MOTOR2_ID);
        motorGroup = new MotorGroup(motor1, motor2);
        motorGroup.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        motorGroup.setMotorPosition(0);
        climberMechanism = new Arm(motorGroup, motorGroup::getMotorPosition, VELOCITY_SOFTLIMIT, GAINS, MASS);
        kSetpoint = () -> 0;
        atPosition = new Trigger(() -> ((kSetpoint.getAsDouble() - motorGroup.getMotorPosition()) < TOLERANCE));
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return climberMechanism.manualCommand(voltageSupplier, this);
    }

    public BooleanSupplier isAtPosition() {
        return atPosition;
    }

    public Command retractClimber() {
        return null;
    }

    public void setSetpoint(double setpoint) {
        kSetpoint = () -> setpoint;
    }

//    public Command goToSetpointCommand() {
//        return climberMechanism.anglePositionControlCommand(kSetpoint, (atPosition) -> , )
//                }
}
