package frc.excalib.control.motor.controllers;

import com.revrobotics.spark.SparkMax;

/**
 * Real hardware implementation of SparkMaxMotorIO using the actual SparkMaxMotor controller.
 * This class bridges the AdvantageKit IO pattern with the excalib SparkMaxMotor implementation.
 */
public class SparkMaxMotorIOReal extends SparkMaxMotorIO {
    private final SparkMaxMotor motor;

    /**
     * Creates a new SparkMaxMotorIOReal instance.
     *
     * @param deviceId The CAN ID of the motor controller
     * @param type The motor type (brushed or brushless)
     */
    public SparkMaxMotorIOReal(int deviceId, SparkMax.MotorType type) {
        this.motor = new SparkMaxMotor(deviceId, type);
    }

    /**
     * Creates a new SparkMaxMotorIOReal instance with an existing motor.
     *
     * @param motor The existing SparkMaxMotor to wrap
     */
    public SparkMaxMotorIOReal(SparkMaxMotor motor) {
        this.motor = motor;
    }

    @Override
    public void updateInputs(SparkMaxMotorInputsAutoLogged inputs) {
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
     * Gets the underlying SparkMaxMotor instance for advanced configuration.
     *
     * @return The SparkMaxMotor instance
     */
    public SparkMaxMotor getMotor() {
        return motor;
    }
}
