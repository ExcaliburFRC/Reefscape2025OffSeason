package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import monologue.Annotations;
import monologue.Logged;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;

public class ArmSubsystem extends SubsystemBase implements Logged {

    // == motors ==
    private final TalonFXMotor firstMotor;
    private final CANcoder m_canCoder;

    private final DoubleSupplier angleSupplier;
    private DoubleSupplier elevatorHeightSupplier;

    private ArmPosition currentState; //
    private final Arm armMechanism;

    private final Trigger toleranceTrigger; //
    private final ContinuousSoftLimit softLimit; //

    private BooleanSupplier isIntakeOpen; //

    public ArmSubsystem() {
        currentState = ArmPosition.DEFAULT_WITHOUT_GAME_PIECE;

        firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);

        m_canCoder = new CANcoder(CAN_CODER_ID);

        angleSupplier = () -> m_canCoder.getPosition().getValueAsDouble() * ROTATIONS_TO_RAD;

        armMechanism = new Arm(
                firstMotor,
                angleSupplier,
                VELOCITY_LIMIT,
                new Gains(),
                new Mass(() -> 0, () -> 0, 1)
        );

        toleranceTrigger = new Trigger(
                () -> (Math.abs(angleSupplier.getAsDouble() - currentState.getAngle()) < TOLERANCE)).debounce(0.1);

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = () -> false;

        firstMotor.setInverted(FORWARD);

        firstMotor.setMotorPosition(angleSupplier.getAsDouble());

        firstMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        firstMotor.setPositionConversionFactor(RPS_TO_RAD_PER_SEC);

        softLimit = new ContinuousSoftLimit(
                () -> {
                    return elevatorHeightSupplier.getAsDouble() < ARM_COLISION_ELEVATOR_LENGTH ? 0 : -Math.PI / 2;
                },
                () -> {
                    return elevatorHeightSupplier.getAsDouble() < ARM_COLISION_ELEVATOR_LENGTH ? Math.PI : Math.PI * 1.5;
                }
        );

        setDefaultCommand(
                armMechanism.anglePositionControlCommand(
                        () -> softLimit.getSetpoint(angleSupplier.getAsDouble(), currentState.getAngle()),
                        (__) -> toleranceTrigger.getAsBoolean(),
                        TOLERANCE,
                        this
                )
        );
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public void setState(ArmPosition state) {
        currentState = state;
    }

    public Command coastCommand() {
        return new StartEndCommand(
                () -> firstMotor.setIdleState(COAST),
                () -> firstMotor.setIdleState(BRAKE),
                this
        ).ignoringDisable(true).withName("Arm Coast Command");
    }

    public DoubleSupplier getAngleSupplier() {
        return angleSupplier;
    }

    public void setElevatorHeightSupplier(DoubleSupplier elevatorHeightSupplier) {
        this.elevatorHeightSupplier = elevatorHeightSupplier;
    }

    public void setIntakeOpen(BooleanSupplier intakeOpen) {
        isIntakeOpen = intakeOpen;
    }

    @Annotations.Log.NT
    public BooleanSupplier isAtPosition() {
        return toleranceTrigger;
    }




}

