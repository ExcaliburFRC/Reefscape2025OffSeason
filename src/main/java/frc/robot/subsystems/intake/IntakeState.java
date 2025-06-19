package frc.robot.subsystems.intake;

public enum IntakeState {
    FLOOR_INTAKE(0,0,0),
    DEFAULT(0,0,0),
    HANDOFF(0,0,0),
    L1_SCORE(0, 0, 0),
    GET_CORAL_BACK_FROM_ARM(0, 0, 0),
    EJECT_CORAL(0, 0, 0);
    final double intakeAngle;
    final double rollerVoltage;
    final double centraliserVoltage;
    private IntakeState(int intakeAngle, int rollerVoltage, int centraliserVoltage){
        this.intakeAngle = intakeAngle;
        this.rollerVoltage = rollerVoltage;
        this.centraliserVoltage = centraliserVoltage;
    }
}
