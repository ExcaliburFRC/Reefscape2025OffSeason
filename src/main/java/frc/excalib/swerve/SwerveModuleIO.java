package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
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
        // Default implementation provides a lightweight simulation so example subsystems
        // using the plain SwerveModuleIO in simulation will see changing sensor values.
        double now = Timer.getFPGATimestamp();
        double dt = now - m_lastTimestamp;
        if (m_lastTimestamp <= 0) dt = 0.0;

        // integrate simulated drive position
        m_simDrivePosition += m_simDriveVelocity * dt;

        // Populate the AutoLogged inputs object
        inputs.drivePositionMeters = m_simDrivePosition;
        inputs.driveVelocityMetersPerSec = m_simDriveVelocity;
        inputs.driveAppliedVolts = m_simDriveAppliedVolts;
        inputs.driveCurrentAmps = m_simDriveCurrent;
        inputs.driveTempCelsius = m_simDriveTemp;

        inputs.turnAbsolutePositionRad = m_simTurnAbsolutePosition;
        inputs.turnPositionRad = m_simTurnPosition;
        inputs.turnVelocityRadPerSec = m_simTurnVelocity;
        inputs.turnAppliedVolts = m_simTurnAppliedVolts;
        inputs.turnCurrentAmps = m_simTurnCurrent;
        inputs.turnTempCelsius = m_simTurnTemp;

        m_lastTimestamp = now;
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
        // In simulation store the velocity so updateInputs can integrate position
        m_simDriveVelocity = velocityMetersPerSec;
    }

    /**
     * Sets the turn motor position setpoint.
     *
     * @param angleRad The desired angle in radians
     */
    public void setTurnPosition(double angleRad) {
        // In simulation store requested turn position (simple model: instant)
        m_simTurnPosition = angleRad;
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        m_simDriveVelocity = 0.0;
        m_simDriveAppliedVolts = 0.0;
        m_simTurnAppliedVolts = 0.0;
    }

    // --- Simulation backing fields ---
    private double m_simDrivePosition = 0.0;
    private double m_simDriveVelocity = 0.0;
    private double m_simDriveAppliedVolts = 0.0;
    private double m_simDriveCurrent = 0.0;
    private double m_simDriveTemp = 20.0;

    private double m_simTurnAbsolutePosition = 0.0;
    private double m_simTurnPosition = 0.0;
    private double m_simTurnVelocity = 0.0;
    private double m_simTurnAppliedVolts = 0.0;
    private double m_simTurnCurrent = 0.0;
    private double m_simTurnTemp = 20.0;

    private double m_lastTimestamp = -1.0;
}
