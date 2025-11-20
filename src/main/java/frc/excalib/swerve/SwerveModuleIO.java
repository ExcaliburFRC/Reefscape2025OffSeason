package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for swerve modules with AdvantageKit logging support.
 * This follows the AdvantageKit IO pattern for hardware abstraction and logging.
 */
public class SwerveModuleIO {

    @AutoLog
    public static class SwerveModuleInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;
        
        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs for this swerve module.
     * This method should be called periodically to refresh sensor readings.
     *
     * @param inputs The inputs object to populate with current sensor values
     */
    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        // Default implementation does nothing (for simulation/replay)
    }

    /**
     * Sets the drive motor voltage.
     *
     * @param volts The voltage to apply to drive motor
     */
    public void setDriveVoltage(double volts) {
        // Default implementation does nothing
    }

    /**
     * Sets the turn motor voltage.
     *
     * @param volts The voltage to apply to turn motor
     */
    public void setTurnVoltage(double volts) {
        // Default implementation does nothing
    }

    /**
     * Sets the drive motor velocity setpoint.
     *
     * @param velocityMetersPerSec The desired velocity in m/s
     */
    public void setDriveVelocity(double velocityMetersPerSec) {
        // Default implementation does nothing
    }

    /**
     * Sets the turn motor position setpoint.
     *
     * @param angleRad The desired angle in radians
     */
    public void setTurnPosition(double angleRad) {
        // Default implementation does nothing
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        // Default implementation does nothing
    }
}
