package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.*;
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
    private final ContinuousSoftLimit softLimit; //

    private BooleanSupplier isIntakeOpen; //

    public ArmSubsystem() {
        currentState = ArmPosition.DEFAULT_WITH_GAME_PIECE;

        firstMotor = new TalonFXMotor(FIRST_MOTOR_ID);

        canCoder = new CANcoder(CAN_CODER_ID);

        firstMotor.setInverted(REVERSE);
        firstMotor.setNeutralMode(NeutralModeValue.Brake);

        firstMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        firstMotor.setPositionConversionFactor((1 / 15.8611544) * Math.PI * 2);

        angleSupplier = () -> (canCoder.getPosition().getValueAsDouble() * Math.PI * 2);
        firstMotor.setMotorPosition(canCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);

        armGains = new Gains(1.8, 0, 0.2, 0, 0, 0, 0.68);
        armMechanism = new Arm(
                firstMotor,
                angleSupplier,
                VELOCITY_LIMIT,
                armGains,
                new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), 1)
        );

        toleranceTrigger = new Trigger(
                () -> (Math.abs(currentState.getAngle() - angleSupplier.getAsDouble()) < Math.PI / 50));

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = () -> false;

        softLimit = new ContinuousSoftLimit(
                () -> {
                    if (elevatorHeightSupplier.getAsDouble() > 0.93) {
                        return -8.3;
                    } else if (elevatorHeightSupplier.getAsDouble() > 0.67) {
                        return -0.82;
                    } else if (elevatorHeightSupplier.getAsDouble() > 0.36) {
                        return 0;

                    } else if (elevatorHeightSupplier.getAsDouble() > 0.22) {
                        return 0.607;
//                    } else if (elevatorHeightSupplier.getAsDouble() > 0.4 && isIntakeOpen.getAsBoolean() == true){
//                        return 0.44;
//                    } else if (elevatorHeightSupplier.getAsDouble() > 0.59 && isIntakeOpen.getAsBoolean() == true) {
//                        return -0.77;
                    }
                    {

                    }
                    return 0.9;
                },
                () -> {
                    if (elevatorHeightSupplier.getAsDouble() > 0.93) {
                        return 6.7;
                    } else if (elevatorHeightSupplier.getAsDouble() > 0.67) {
                        return 3.8;
                    } else if (elevatorHeightSupplier.getAsDouble() > 0.36) {
                        return 3;
                    } else if (elevatorHeightSupplier.getAsDouble() > 0.22) {
                        return 2.3;
//                    } else if (elevatorHeightSupplier.getAsDouble()> 0.4 && isIntakeOpen.getAsBoolean()) {
//                        return 3;
//                    } else if (elevatorHeightSupplier.getAsDouble() > 0.59 && isIntakeOpen.getAsBoolean()) {
//                        return 3.8;d
                    }
                    return 2;
                }
        );

        setDefaultCommand((goToStateCommand()));

    }

    public Command goToStateCommand() {
        return armMechanism.anglePositionControlCommand(
                () -> softLimit.getSetpoint(angleSupplier.getAsDouble(), currentState.getAngle()),
                (at) -> at = false,
                Math.PI / 50,
                this
        ).until(this::isAtPosition);
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public Command setStateCommand(ArmPosition state) {
        return new InstantCommand(() -> currentState = state, this);
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
    public boolean isAtPosition() {
        return toleranceTrigger.getAsBoolean();
    }

    @NT
    public boolean isInTolernace() {
        return Math.abs(currentState.getAngle() - angleSupplier.getAsDouble()) < Math.PI / 50;
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

    @NT
    public double getCurrent() {
        return armMechanism.logCurrent();
    }

    @NT
    public double getVoltage() {
        return armMechanism.logVoltage();
    }

    @NT
    public double getLimitedSetpoint() {
        return softLimit.getSetpoint(angleSupplier.getAsDouble(), softLimit.limit(currentState.getAngle()));
    }

    @NT
    public double getElevatorHeightSupplie(){
        return elevatorHeightSupplier.getAsDouble();
    }

}
