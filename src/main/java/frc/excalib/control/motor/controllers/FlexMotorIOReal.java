package frc.excalib.control.motor.controllers;

import com.revrobotics.spark.SparkFlex;

/**
 * Real hardware implementation of FlexMotorIO using the actual FlexMotor controller.
 * This class bridges the AdvantageKit IO pattern with the excalib FlexMotor implementation.
 */
public class FlexMotorIOReal extends FlexMotorIO {
    private final FlexMotor motor;

    /**
     * Creates a new FlexMotorIOReal instance.
     *
     * @param deviceId The CAN ID of the motor controller
     * @param type The motor type (brushed or brushless)
     */
    public FlexMotorIOReal(int deviceId, SparkFlex.MotorType type) {
        this.motor = new FlexMotor(deviceId, type);
    }

    /**
     * Creates a new FlexMotorIOReal instance with an existing motor.
     *
     * @param motor The existing FlexMotor to wrap
     */
    public FlexMotorIOReal(FlexMotor motor) {
        this.motor = motor;
    }

    @Override
    public void updateInputs(FlexMotorInputsAutoLogged inputs) {
        inputs.positionRotations = motor.getMotorPosition();
        inputs.velocityRotationsPerSecond = motor.getMotorVelocity();
        inputs.appliedVolts = motor.getVoltage();
        inputs.currentAmps = motor.getCurrent();
        inputs.temperatureCelsius = motor.getTemperature();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPercentage(double percentage) {
        motor.setPercentage(percentage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPosition(double positionRotations) {
        motor.setMotorPosition(positionRotations);
    }

    /**
     * Gets the underlying FlexMotor instance for advanced configuration.
     *
     * @return The FlexMotor instance
     */
    public FlexMotor getMotor() {
        return motor;
    }
}
