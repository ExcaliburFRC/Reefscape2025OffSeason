package frc.excalib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Real hardware implementation of SwerveModuleIO using the excalib SwerveModule.
 * This class bridges the AdvantageKit IO pattern with the existing SwerveModule implementation.
 */
public class SwerveModuleIOReal extends SwerveModuleIO {
    private final SwerveModule module;

    /**
     * Creates a new SwerveModuleIOReal instance wrapping an existing SwerveModule.
     *
     * @param module The existing SwerveModule to wrap
     */
    public SwerveModuleIOReal(SwerveModule module) {
        this.module = module;
    }

    @Override
    public void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        // Drive wheel telemetry
        inputs.drivePositionMeters = module.m_driveWheel.logPosition();
        inputs.driveVelocityMetersPerSec = module.m_driveWheel.logVelocity();
        inputs.driveAppliedVolts = module.m_driveWheel.logVoltage();
        inputs.driveCurrentAmps = module.m_driveWheel.logCurrent();
        inputs.driveTempCelsius = 0.0; // Not available from FlyWheel
        
        // Turret (turn) telemetry
        inputs.turnAbsolutePositionRad = module.m_turret.getPosition().getRadians();
        inputs.turnPositionRad = module.m_turret.getPosition().getRadians();
        inputs.turnVelocityRadPerSec = module.m_turret.logVelocity();
        inputs.turnAppliedVolts = module.m_turret.logVoltage();
        inputs.turnCurrentAmps = module.m_turret.logCurrent();
        inputs.turnTempCelsius = 0.0; // Not available from Turret
    }

    @Override
    public void setDriveVoltage(double volts) {
        module.m_driveWheel.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        module.m_turret.setVoltage(volts);
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
        // Use the drive wheel's velocity control
        module.m_driveWheel.setVoltage(velocityMetersPerSec * 0.2); // Simple feedforward
    }

    @Override
    public void setTurnPosition(double angleRad) {
        module.m_turret.setPosition(new Rotation2d(angleRad));
    }

    @Override
    public void stop() {
        module.m_driveWheel.setOutput(0);
        module.m_turret.setOutput(0);
    }

    /**
     * Gets the underlying SwerveModule instance for advanced configuration.
     *
     * @return The SwerveModule instance
     */
    public SwerveModule getModule() {
        return module;
    }
}
