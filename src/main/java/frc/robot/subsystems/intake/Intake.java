package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.mechanisms.Arm.Arm;
import frc.excalib.mechanisms.Mechanism;

import java.lang.reflect.InaccessibleObjectException;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

    public final SparkMaxMotor angleMotor, rollerMotor;
    public final CANcoder angleEncoder;
    public final DoubleSupplier angleSupplier;
    public final DigitalInput limitSwitch;
    public final Arm armMechanism;
    public final Mechanism rollerMechanism;
    private final Trigger hasNoteTrigger;
    private final Trigger inStateTrigger;
    private final SoftLimit velocitySoftLimit;
    private final Gains armGains;
    private final Mass armMass;
    private IntakeStates currentState;



    public Intake() {
        this.angleMotor = new SparkMaxMotor(IntakeConstants.ANGLE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.rollerMotor = new SparkMaxMotor(IntakeConstants.ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.angleEncoder = new CANcoder(IntakeConstants.ANGLE_MOTOR_ID);
        this.angleSupplier = () -> ((angleEncoder.getAbsolutePosition().getValueAsDouble() * IntakeConstants.ROTATIONS_TO_RAD));
        this.limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_CHANNEL);
        this.velocitySoftLimit = new SoftLimit(
                () -> IntakeConstants.ANGLE_VELOCITY_MIN_LIMIT,
                () -> IntakeConstants.ANGLE_VELOCITY_MAX_LIMIT
        );
        this.armMass = new Mass(() -> Math.acos(angleSupplier.getAsDouble()), () -> Math.asin(angleSupplier.getAsDouble()), IntakeConstants.INTAKE_MASS);
        this.armGains = new Gains();
        this.armMechanism = new Arm(angleMotor, this.angleSupplier, this.velocitySoftLimit, this.armGains, this.armMass);
        this.rollerMechanism = new Mechanism(rollerMotor);
        this.hasNoteTrigger = new Trigger(limitSwitch::get);
        this.currentState = IntakeStates.DEFAULT;
        this.inStateTrigger = new Trigger(() -> (
                (Math.abs(this.currentState.getArmAngle() - angleSupplier.getAsDouble()) < IntakeConstants.TOLERANCE)
                ));
    }

    public void setCurrentState(IntakeStates state) {
        this.currentState = state;
    }

    public Command setRollerVoltage(double voltage) {
        return new RunCommand(
                () -> this.rollerMechanism.setVoltage(voltage)
        );
    }

    public Command goToState(IntakeStates state) {
        return new ParallelCommandGroup(
                setRollerVoltage(state.getRollerVoltage()),
                this.armMechanism.goToAngleCommand(state.getArmAngle(), (h) -> h = true, IntakeConstants.TOLERANCE)
        ).until(this.inStateTrigger);
    }
}