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

    // === Other ===
    private Gains armGains;
    private final ContinuousSoftLimit softLimit;
    private ContinuousSoftLimit limitHelper;


    public ArmSubsystem() {
        currentState = ArmPosition.DEFAULT_WITH_GAME_PIECE;

        armMotor = new TalonFXMotor(FIRST_MOTOR_ID);

        canCoder = new CANcoder(CAN_CODER_ID);

        armMotor.setInverted(REVERSE);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.setVelocityConversionFactor(RPS_TO_RAD_PER_SEC);
        armMotor.setPositionConversionFactor(((1 / 15.8611544) * Math.PI * 2) / 1.0755744 * 0.993157566);

        angleSupplier = () -> (canCoder.getPosition().getValueAsDouble() * Math.PI * 2);

        armMotor.setMotorPosition(angleSupplier.getAsDouble());

        elevatorHeightSupplier = () -> 0;
        isIntakeOpen = new Trigger(() -> false);

        armMechanism = new Arm(armMotor, angleSupplier, VELOCITY_LIMIT, new Gains(1.8, 0, 0.2, 0, 0, 0, 1.2), new Mass(() -> Math.cos(angleSupplier.getAsDouble()), () -> Math.sin(angleSupplier.getAsDouble()), 1));

        atPostionTrigger = new Trigger(() -> (Math.abs(getLimitedSetpoint() - angleSupplier.getAsDouble()) < POSITION_TOLERANCE_RAD));

        limitHelper = new ContinuousSoftLimit(() -> -8.3, () -> 6.7);


        softLimit = new ContinuousSoftLimit(
                () -> {
                    double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
                    if (heightDiff > ARM_LENGTH) {
                        return -8.3;
                    }


                    return limitHelper.getSetpoint(
                            angleSupplier.getAsDouble(), Math.PI / 2) + getMin() - Math.PI / 2;
                },
                () -> {
                    double heightDiff = elevatorHeightSupplier.getAsDouble() - INTAKE_HEIGHT;
                    if (heightDiff > ARM_LENGTH) {
                        return 6.7;
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
        return armMechanism.anglePositionControlCommand(() -> softLimit.getSetpoint(angleSupplier.getAsDouble(), currentState.getAngle()), (at) -> at = false, POSITION_TOLERANCE_RAD, this).until(this::isAtPosition);
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

    @NT
    public boolean isInLimit() {
        return softLimit.within(currentState.getAngle());
    }

    @NT
    public double getError() {
        return currentState.getAngle() - angleSupplier.getAsDouble();
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
