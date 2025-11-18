package frc.excalib.control.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for IMU/Gyroscope with AdvantageKit logging support.
 * This follows the AdvantageKit IO pattern for hardware abstraction and logging.
 */
public class IMUIO {

    @AutoLog
    public static class IMUInputs {
        public double yawDegrees = 0.0;
        public double pitchDegrees = 0.0;
        public double rollDegrees = 0.0;
        public double accelerationXGs = 0.0;
        public double accelerationYGs = 0.0;
        public double accelerationZGs = 0.0;
        public boolean connected = false;
    }

    /**
     * Updates the set of loggable inputs for this IMU.
     * This method should be called periodically to refresh sensor readings.
     *
     * @param inputs The inputs object to populate with current sensor values
     */
    public void updateInputs(IMUInputsAutoLogged inputs) {
        // Default implementation does nothing (for simulation/replay)
    }

    /**
     * Resets the IMU yaw to zero.
     */
    public void reset() {
        // Default implementation does nothing
    }

    /**
     * Sets the yaw angle of the IMU.
     *
     * @param yaw The yaw angle to set
     */
    public void setYaw(Rotation2d yaw) {
        // Default implementation does nothing
    }
}
