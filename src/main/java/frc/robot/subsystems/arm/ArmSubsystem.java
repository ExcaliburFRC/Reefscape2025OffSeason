package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

import java.util.function.DoubleSupplier;

import static frc.excalib.control.motor.motor_specs.DirectionState.*;
import static frc.excalib.control.motor.motor_specs.IdleState.BRAKE;
import static frc.excalib.control.motor.motor_specs.IdleState.COAST;
import static frc.robot.subsystems.arm.Constants.*;
import static frc.robot.subsystems.intake.Constants.POSITION_TOLERANCE_RAD;

public class ArmSubsystem extends SubsystemBase implements Logged {

    // === Hardware ===
    private final TalonFXMotor armMotor;
    private final CANcoder canCoder;
    private final Arm armMechanism;

    // === Suppliers ===
    private final DoubleSupplier angleSupplier;
    private DoubleSupplier elevatorHeightSupplier;
    private Trigger isIntakeOpen;

    // === Triggers and States ===
    private final Trigger atPostionTrigger;
    private ArmPosition currentState;
    private final CurrentLimitsConfigs limitsConfigs;

    // === Other ===
    private Gains armGains;
    private final ContinuousSoftLimit softLimit;
    private ContinuousSoftLimit limitHelper;

//    private Trigger mirrorArmSetpoint;


    public ArmSubsystem() {
        currentState = ArmPosition.UPWARDS;

        armMotor = new TalonFXMotor(FIRST_MOTOR_ID);

        canCoder = new CANcoder(CAN_CODER_ID);

        armMotor.setInverted(REVERSE);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        armMotor.setPositionConversionFactor(((1 / 15.8611544) * Math.PI * 2) / 1.0755744 * 0.993157566);

        angleSupplier = () -> (canCoder.getPosition().getValueAsDouble() * Math.PI * 2);

        armMotor.setMotorPosition(angleSupplier.getAsDouble());

        limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 45;
        limitsConfigs.SupplyCurrentLimitEnable = true;

        armMotor.getConfigurator().apply(limitsConfigs);

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = new Trigger(() -> false);

        armMechanism = new Arm(
                armMotor,
                angleSupplier,
                VELOCITY_LIMIT,
                new Gains(3, 0, 0.1, 0, 0, 0, 0.8),
                new Mass(
                        () -> Math.cos(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS_OFFSET),
                        () -> Math.sin(angleSupplier.getAsDouble() + ARM_MASS_TO_AXIS_OFFSET),
                        1
                )
        );

        atPostionTrigger = new Trigger(
                () -> (
                        Math.abs(getLimitedSetpoint() - angleSupplier.getAsDouble()) < POSITION_TOLERANCE_RAD));

        limitHelper = new ContinuousSoftLimit(() -> -MAX_SCORE_RAD, () -> 3 * Math.PI + MAX_SCORE_RAD);

        softLimit = new ContinuousSoftLimit(
                () -> {
                    double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
                    if (heightDiff < 0) {
                        return limitHelper.getSetpoint(
                                angleSupplier.getAsDouble(), Math.PI / 2) + 1.1 - Math.PI / 2;
                    }
                    if (heightDiff > ARM_LENGTH) {
                        return limitHelper.getMinLimit();

                    }
                    return limitHelper.getSetpoint(
                            angleSupplier.getAsDouble(), Math.PI / 2) + getMin() - Math.PI / 2;
                },
                () -> {
                    double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
                    if (heightDiff < 0) {
                        return limitHelper.getSetpoint(
                                angleSupplier.getAsDouble(), Math.PI / 2) + 1.8 - Math.PI / 2;
                    }
                    if (heightDiff > ARM_LENGTH) {
                        return limitHelper.getMaxLimit();
                    }

                    return limitHelper.getSetpoint(
                            angleSupplier.getAsDouble(), Math.PI / 2) + getMax() - Math.PI / 2;
                }
        );

        setDefaultCommand((goToStateCommand()));
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        return armMechanism.manualCommand(voltageSupplier, this);
    }

    public Command goToStateCommand() {
        return armMechanism.anglePositionControlCommand(
                () -> softLimit.limit(
                        limitHelper.getSetpoint(
                                angleSupplier.getAsDouble(),
                                currentState.getAngle()
                        )
                ),
                (at) -> at = false,
                POSITION_TOLERANCE_RAD,
                this
        ).until(this::isAtPosition);
    }

    public Command setStateCommand(ArmPosition state) {
        return new InstantCommand(() -> currentState = state);
    }

    public Command coastCommand() {
        return new StartEndCommand(() -> armMotor.setIdleState(COAST), () -> armMotor.setIdleState(BRAKE), this).ignoringDisable(true).withName("Arm Coast Command");
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
        return armMotor.getMotorPosition();
    }

    @NT
    public double getSetpoint() {
        return currentState.getAngle();
    }

    @NT
    public double getLimitedSetpoint() {
        return softLimit.limit(softLimit.getSetpoint(angleSupplier.getAsDouble(), currentState.getAngle()));
    }

    @NT
    public boolean isInLimit() {
        return softLimit.within(currentState.getAngle());
    }

    @NT
    public double getError() {
        return Math.abs(getLimitedSetpoint() - angleSupplier.getAsDouble());
    }

    @NT
    public double getHeightDiff() {
        return elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
    }

    public double getMin() {
        double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
        double minLimit = Math.asin(-heightDiff / ARM_LENGTH);
        return minLimit;
    }

    public double getMax() {
        double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
        double maxLimit = Math.asin(-heightDiff / ARM_LENGTH);
        return Math.PI - maxLimit;
    }
}
