package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.ContinuousSoftLimit;
import frc.excalib.control.math.physics.Mass;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import monologue.Annotations.Log.NT;
import monologue.Logged;
import org.ejml.dense.row.factory.LinearSolverFactory_MT_DDRM;

import javax.swing.plaf.PanelUI;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;
import static frc.robot.subsystems.intake.Constants.POSITION_TOLERANCE_RAD;

public class ArmSubsystem extends SubsystemBase implements Logged {

    // === Hardware ===
    private final TalonFXMotor firstMotor;
    private final CANcoder canCoder;
    private final Arm armMechanism;

    // === Suppliers ===
    private final DoubleSupplier angleSupplier;
    private DoubleSupplier elevatorHeightSupplier;
    private Trigger isIntakeOpen;

    // === Triggers and States ===
    private final Trigger atPostionTrigger;
    private ArmPosition currentState;

    // === Other ===
    private Gains armGains;
    private final ContinuousSoftLimit softLimit;


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

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = new Trigger(() -> false);

        armMechanism = new Arm(
                firstMotor,
                angleSupplier,
                VELOCITY_LIMIT,
                new Gains(1.8, 0, 0.2, 0, 0, 0, 1.2),
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble()),
                        () -> Math.sin(angleSupplier.getAsDouble()),
                        1
                )
        );

        atPostionTrigger = new Trigger(
                () -> (Math.abs(getLimitedSetpoint() - angleSupplier.getAsDouble()) < POSITION_TOLERANCE_RAD)
        );

        softLimit = new ContinuousSoftLimit(
                () -> {
                    double h = elevatorHeightSupplier.getAsDouble();
                    if (isIntakeOpen.getAsBoolean()) {
                        if (h > 0.90) {
                            return -8.3;
                        } else if (h > 0.67) {
                            return -0.82 + applyRotationSlack();
                        } else if (h > 0.36) {
                            return 0 + applyRotationSlack();
                        } else if (h > 0.22) {
                            return 0.607 + applyRotationSlack();
                        }
                        return 0.9 + applyRotationSlack();
                    } else {
                        if (h > 0.87) {
                            return -8.3;
                        } else if (h > 0.71 + 0.08) {
                            return -0.81 + applyRotationSlack();
                        } else if (h > 0.45+0.08) {
                            return applyRotationSlack();
                        }
                        return 1.1 + applyRotationSlack();
                    }
                },
                () -> {
                    double h = elevatorHeightSupplier.getAsDouble();
                    if (isIntakeOpen.getAsBoolean()) {
                        if (h > 0.93) {
                            return 6.7;
                        } else if (h > 0.67) {
                            return 3.8 + applyRotationSlack();
                        } else if (h > 0.36) {
                            return 3 + applyRotationSlack();
                        } else if (h > 0.22) {
                            return 2.3 + applyRotationSlack();
                        }
                        return 2 + applyRotationSlack();
                    } else {
                        if (h > 0.87) {
                            return 6.7;
                        } else if (h > 0.71+0.08) {
                            return -2.84 + applyRotationSlack();
                        } else if (h > 0.45+0.08) {
                            return -0.283 + applyRotationSlack();
                        } else {
                            return 1.6 + applyRotationSlack();
                        }
                    }
                }
        );
        setDefaultCommand((goToStateCommand()));
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public Command goToStateCommand() {
        return armMechanism.anglePositionControlCommand(
                () -> softLimit.getSetpoint(angleSupplier.getAsDouble(), currentState.getAngle()),
                (at) -> at = false,
                POSITION_TOLERANCE_RAD,
                this
        ).until(this::isAtPosition);
    }

    public Command setStateCommand(ArmPosition state) {
        return new InstantCommand(() -> currentState = state);
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

    public void setIntakeOpen(Trigger intakeOpen) {
        isIntakeOpen = intakeOpen;
    }

    @NT
    public boolean isAtPosition() {
        return atPostionTrigger.getAsBoolean();
    }

    @NT
    public boolean isIntakeOpen() {
        return isIntakeOpen.getAsBoolean();
    }

    @NT
    public double getEncoderValue() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @NT
    public double getMotorPosition() {
        return firstMotor.getMotorPosition();
    }

    @NT
    public double getSetpoint() {
        return currentState.getAngle();
    }

    @NT
    public double getLimitedSetpoint() {
        return softLimit.getSetpoint(angleSupplier.getAsDouble(), softLimit.limit(currentState.getAngle()));
    }

    @NT
    public double applyRotationSlack() {
        if (angleSupplier.getAsDouble() < 0) {
            DoubleSupplier tempSupplier = () -> angleSupplier.getAsDouble() + 20 * Math.PI;
            double temp = (int) tempSupplier.getAsDouble() / (Math.PI * 2) * 2 * Math.PI;
            temp -= 20 * Math.PI;
        }
        return (int) (angleSupplier.getAsDouble() / (Math.PI * 2)) * 2 * Math.PI;
    }


}
