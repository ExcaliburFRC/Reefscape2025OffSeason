package frc.excalib.control.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Real hardware implementation of IMUIO using the NavX gyroscope.
 * This class bridges the AdvantageKit IO pattern with the excalib NavX implementation.
 */
public class NavXIOReal extends IMUIO {
    private final NavX navx;

    /**
     * Creates a new NavXIOReal instance.
     *
     * @param offsetRotation The offset rotation for the IMU
     */
    public NavXIOReal(Rotation3d offsetRotation) {
        this.navx = new NavX(offsetRotation);
    }

    /**
     * Creates a new NavXIOReal instance with an existing NavX.
     *
     * @param navx The existing NavX to wrap
     */
    public NavXIOReal(NavX navx) {
        this.navx = navx;
    }

    @Override
    public void updateInputs(IMUInputsAutoLogged inputs) {
        inputs.yawDegrees = navx.getZRotation().getDegrees();
        inputs.pitchDegrees = navx.getYRotation().getDegrees();
        inputs.rollDegrees = navx.getXRotation().getDegrees();
        inputs.accelerationXGs = navx.getAccX() / 9.8;
        inputs.accelerationYGs = navx.getAccY() / 9.8;
        inputs.accelerationZGs = navx.getAccZ() / 9.8;
        inputs.connected = navx.isConnected();
    }

    @Override
    public void reset() {
        navx.resetIMU();
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        navx.setRotation(yaw);
    }

    /**
     * Gets the underlying NavX instance for advanced configuration.
     *
     * @return The NavX instance
     */
    public NavX getNavX() {
        return navx;
    }
}
