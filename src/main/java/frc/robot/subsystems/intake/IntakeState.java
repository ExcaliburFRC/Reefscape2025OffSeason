package frc.robot.subsystems.intake;

public enum IntakeState {
    FLOOR_INTAKE(0,0,0),
    STOW(0,0,0),
    PASS_TO_ARM(0,0,0);
    final double intakeAngle;
    final double rollerVoltage;
    final double centraliserVoltage;
    private IntakeState(int intakeAngle, int rollerVoltage, int centraliserVoltage){
        this.intakeAngle = intakeAngle;
        this.rollerVoltage = rollerVoltage;
        this.centraliserVoltage = centraliserVoltage;
    }
}
