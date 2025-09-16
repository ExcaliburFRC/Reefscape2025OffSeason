package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.gains.SysidConfig;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import monologue.Annotations;
import monologue.Annotations.Log.NT;
import monologue.Logged;
import org.opencv.core.Mat;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;

public class ArmSubsystem extends SubsystemBase implements Logged {

    // == motors ==
    private final TalonFXMotor firstMotor;
    private final CANcoder canCoder;

    private final DoubleSupplier angleSupplier;
    private DoubleSupplier elevatorHeightSupplier;
    private Gains armGains;

    private ArmPosition currentState; //
    private final Arm armMechanism;

    private final Trigger toleranceTrigger; //
    private final SoftLimit softLimit; //

    private BooleanSupplier isIntakeOpen; //

    public ArmSubsystem() {
        currentState = ArmPosition.DEFAULT_WITHOUT_GAME_PIECE;

        firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);
        canCoder = new CANcoder(CAN_CODER_ID);

        firstMotor.setInverted(REVERSE);
        firstMotor.setNeutralMode(NeutralModeValue.Brake);

        firstMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        firstMotor.setPositionConversionFactor((1 / 15.8611544) * Math.PI * 2);

        angleSupplier = () -> (canCoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
        firstMotor.setMotorPosition(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
        //-0.273923 rotations
        // 4.344735


        armGains = new Gains(1.4, 0, 0.03, 0, 0, 0, 0.6);
        armMechanism = new Arm(
                firstMotor,
                angleSupplier,
                VELOCITY_LIMIT,
                armGains,
                new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), 1)
        );

        toleranceTrigger = new Trigger(
                () -> (Math.abs(angleSupplier.getAsDouble() - currentState.getAngle()) < TOLERANCE));

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = () -> false;

        softLimit = new SoftLimit(
                () -> {
                    return elevatorHeightSupplier.getAsDouble() < ARM_COLISION_ELEVATOR_LENGTH ? 0 : -Math.PI / 2;
                },
                () -> {
                    return elevatorHeightSupplier.getAsDouble() < ARM_COLISION_ELEVATOR_LENGTH ? Math.PI : Math.PI * 1.5;
                }
        );

        setDefaultCommand(setStateCommand(ArmPosition.CHECK1).withTimeout(0.1).andThen(goToStateCommand()));

//        setDefaultCommand(
//
//
//        );
    }

    public Command goToStateCommand() {
        return armMechanism.anglePositionControlCommand(
                () -> currentState.getAngle(),
                (at) -> at = false,
                Math.PI/50,
                this
        );
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public Command setStateCommand(ArmPosition state) {
        return new RunCommand(() -> currentState = state, this);
    }

    public Command coastCommand() {
        return new StartEndCommand(
                () -> firstMotor.setIdleState(COAST),
                () -> firstMotor.setIdleState(BRAKE),
                this
        ).ignoringDisable(true).withName("Arm Coast Command");
    }


    @NT
    public double getAngleSupplier() {
        return angleSupplier.getAsDouble();
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public void setIntakeOpen(BooleanSupplier intakeOpen) {
        isIntakeOpen = intakeOpen;
    }

    @NT
    public BooleanSupplier isAtPosition() {
        return toleranceTrigger;
    }

    @NT
    public boolean isInTolernace() {
        return Math.abs(currentState.getAngle() - angleSupplier.getAsDouble()) < Math.PI/50;
    }

    @NT
    public double getMotorPowwer() {
        return 1 * Math.cos(angleSupplier.getAsDouble());
    }

    @NT
    public double getEncoderValue() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @NT
    public double getMotorValue() {
        return firstMotor.getMotorPosition();

    }

    @NT
    public double getSetpoint() {
        return currentState.getAngle();
    }


}

