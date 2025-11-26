package frc.excalib.control.motor.controllers;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for SparkFlex motors with AdvantageKit logging support.
 * This follows the AdvantageKit IO pattern for hardware abstraction and logging.
 */
public class FlexMotorIO {

    @AutoLog
    public static class FlexMotorInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs for this motor.
     * This method should be called periodically to refresh sensor readings.
     *
     * @param inputs The inputs object to populate with current sensor values
     */
    public void updateInputs(FlexMotorInputsAutoLogged inputs) {
        // Default implementation does nothing (for simulation/replay)
    }

    /**
     * Sets the motor output voltage.
     *
     * @param volts The voltage to apply (-12.0 to 12.0)
     */
    public void setVoltage(double volts) {
        // Default implementation does nothing
    }

    /**
     * Sets the motor output percentage.
     *
     * @param percentage The percentage to apply (-1.0 to 1.0)
     */
    public void setPercentage(double percentage) {
        // Default implementation does nothing
    }

    /**
     * Stops the motor.
     */
    public void stop() {
        // Default implementation does nothing
    }

    /**
     * Sets the motor position.
     *
     * @param positionRotations The position in rotations
     */
    public void setPosition(double positionRotations) {
        // Default implementation does nothing
    }
}
