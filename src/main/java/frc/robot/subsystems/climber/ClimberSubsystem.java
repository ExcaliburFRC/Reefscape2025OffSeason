package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.motor.controllers.MotorGroup;
import frc.excalib.control.motor.controllers.SparkMaxMotor;
import frc.excalib.control.motor.controllers.TalonFXMotor;
import frc.excalib.mechanisms.Arm.Arm;
import monologue.Annotations.Log;
import monologue.Logged;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.climber.Constants.*;

public class ClimberSubsystem extends SubsystemBase implements Logged {
    private final TalonFXMotor firstMotor, secondMotor;
    private final SparkMaxMotor rollerMotor;
    private final MotorGroup motorGroup;
    private final Arm climberMechanism;
    private final Trigger atPosition; //
    private DoubleSupplier setpoint; //
    private SoftLimit limit;

    public ClimberSubsystem() {
        firstMotor = new TalonFXMotor(MOTOR1_ID);
        secondMotor = new TalonFXMotor(MOTOR2_ID);

        rollerMotor = new SparkMaxMotor(ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs = new CurrentLimitsConfigs();
        limitsConfigs.SupplyCurrentLimit = 35;
        limitsConfigs.SupplyCurrentLimitEnable = true;
        motorGroup = new MotorGroup(firstMotor, secondMotor);
        firstMotor.getConfigurator().apply(limitsConfigs);
        secondMotor.getConfigurator().apply(limitsConfigs);
        limit = new SoftLimit(() -> 0, () -> 2.6);

        motorGroup.setPositionConversionFactor(ARM_POSITION_CONVERSION_FACTOR);
        motorGroup.setVelocityConversionFactor(ARM_POSITION_CONVERSION_FACTOR);

        motorGroup.setMotorPosition(INITIAL_START_ANGLE);

        climberMechanism = new Arm(motorGroup, motorGroup::getMotorPosition, VELOCITY_SOFTLIMIT, GAINS, MASS);

        setpoint = () -> INITIAL_START_ANGLE;

        atPosition = new Trigger(() -> ((setpoint.getAsDouble() - motorGroup.getMotorPosition()) < TOLERANCE));
    }

    public Command manualCommand(DoubleSupplier voltageSupplier) {
        Command command = climberMechanism.manualCommand(voltageSupplier)
                .alongWith(new RunCommand(()-> rollerMotor.setVoltage(9)));
        command.addRequirements(this);
        return command;
    }

    public Command goToState(double angle) {
        Command command = new ParallelCommandGroup(
                climberMechanism.anglePositionControlCommand(
                        () -> limit.limit(angle),
                        (atPosition) -> atPosition = false,
                        Math.PI / 20
                ).until(atPosition),
                new RunCommand(() -> rollerMotor.setVoltage(3))
        );
        command.addRequirements(this);
        return command;
    }

    @Log.NT
    public DoubleSupplier getSetpoint() {
        return setpoint;
    }

    @Log.NT
    public Trigger getAtPosition() {
        return atPosition;
    }

    public enum States {
        DEFAULT(Math.PI / 2),
        RETRACT(2.1),
        OPEN(0);

        final double angle;

        States(double angle) {
            this.angle = angle;
        }
    }


}
