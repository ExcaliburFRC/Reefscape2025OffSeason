package frc.excalib.control.motor.controllers;

/**
 * Real hardware implementation of TalonFXMotorIO using the actual TalonFXMotor controller.
 * This class bridges the AdvantageKit IO pattern with the excalib TalonFXMotor implementation.
 */
public class TalonFXMotorIOReal extends TalonFXMotorIO {
    private final TalonFXMotor motor;

    /**
     * Creates a new TalonFXMotorIOReal instance.
     *
     * @param deviceId The CAN ID of the motor controller
     */
    public TalonFXMotorIOReal(int deviceId) {
        this.motor = new TalonFXMotor(deviceId);
    }

    /**
     * Creates a new TalonFXMotorIOReal instance with a specific CAN bus.
     *
     * @param deviceId The CAN ID of the motor controller
     * @param canbus The name of the CAN bus
     */
    public TalonFXMotorIOReal(int deviceId, String canbus) {
        this.motor = new TalonFXMotor(deviceId, canbus);
    }

    /**
     * Creates a new TalonFXMotorIOReal instance with an existing motor.
     *
     * @param motor The existing TalonFXMotor to wrap
     */
    public TalonFXMotorIOReal(TalonFXMotor motor) {
        this.motor = motor;
    }

    @Override
    public void updateInputs(TalonFXMotorInputsAutoLogged inputs) {
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
     * Gets the underlying TalonFXMotor instance for advanced configuration.
     *
     * @return The TalonFXMotor instance
     */
    public TalonFXMotor getMotor() {
        return motor;
    }
}
