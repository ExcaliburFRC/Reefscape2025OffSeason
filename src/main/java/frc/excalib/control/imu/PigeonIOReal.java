package frc.excalib.control.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Real hardware implementation of IMUIO using the Pigeon2 gyroscope.
 * This class bridges the AdvantageKit IO pattern with the excalib Pigeon implementation.
 */
public class PigeonIOReal extends IMUIO {
    private final Pigeon pigeon;

    /**
     * Creates a new PigeonIOReal instance.
     *
     * @param deviceId The CAN ID of the Pigeon2
     * @param offsetRotation The offset rotation for the IMU
     */
    public PigeonIOReal(int deviceId, Rotation3d offsetRotation) {
        this.pigeon = new Pigeon(deviceId, offsetRotation);
    }

    /**
     * Creates a new PigeonIOReal instance with a specific CAN bus.
     *
     * @param deviceId The CAN ID of the Pigeon2
     * @param canbus The name of the CAN bus
     * @param offsetRotation The offset rotation for the IMU
     */
    public PigeonIOReal(int deviceId, String canbus, Rotation3d offsetRotation) {
        this.pigeon = new Pigeon(deviceId, canbus, offsetRotation);
    }

    /**
     * Creates a new PigeonIOReal instance with an existing Pigeon.
     *
     * @param pigeon The existing Pigeon to wrap
     */
    public PigeonIOReal(Pigeon pigeon) {
        this.pigeon = pigeon;
    }

    @Override
    public void updateInputs(IMUInputsAutoLogged inputs) {
        inputs.yawDegrees = pigeon.getZRotation().getDegrees();
        inputs.pitchDegrees = pigeon.getYRotation().getDegrees();
        inputs.rollDegrees = pigeon.getXRotation().getDegrees();
        inputs.accelerationXGs = pigeon.getAccX() / 9.8;
        inputs.accelerationYGs = pigeon.getAccY() / 9.8;
        inputs.accelerationZGs = pigeon.getAccZ() / 9.8;
        inputs.connected = true;
    }

    @Override
    public void reset() {
        pigeon.resetIMU();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        pigeon.setRotation(yaw);
    }

    /**
     * Gets the underlying Pigeon instance for advanced configuration.
     *
     * @return The Pigeon instance
     */
    public Pigeon getPigeon() {
        return pigeon;
    }
}
