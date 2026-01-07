package frc.robot.subsystems.intake;

public enum IntakeStates {


    FLOOR_INTAKE(IntakeConstants.FLOOR_INTAKE_ANGLE, IntakeConstants.FLOOR_INTAKE_VOLTAGE),

    SHOOTER_HANDOFF(IntakeConstants.SHOOTER_HANDOFF_ANGLE, IntakeConstants.SHOOTER_HANDOFF_VOLTAGE),

    DEFAULT(IntakeConstants.DEFAULT_ANGLE, IntakeConstants.DEFAULT_VOLTAGE);
    private final double armAngle;
    private final double rollerVoltage;

    IntakeStates(double armAngle, double rollerVoltage) {
        this.armAngle = armAngle;
        this.rollerVoltage = rollerVoltage;
    }

    public double getArmAngle() {
        return this.armAngle;
    }

    public double getRollerVoltage() {
        return this.rollerVoltage;
    }
}
