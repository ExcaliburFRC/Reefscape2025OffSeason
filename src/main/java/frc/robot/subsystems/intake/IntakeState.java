package frc.robot.subsystems.intake;

public enum IntakeState {
    FLOOR_INTAKE(3.18,-10,0),
    CENTERLIZE(0.8635,-0.5,1),
    DEFAULT(0.8635,0,0),
    PREHANDOFF(0.8635,0,0.3),
    HANDOFF(0.8765,9,0.3),
    L1_SCORE(0, 0, 0),
    GET_CORAL_BACK_FROM_ARM(0, 0, 0),
    EJECT_CORAL(2, 0, 0);
    final double intakeAngle;
    final double rollerVoltage;
    final double centraliserVoltage;
    private IntakeState(double intakeAngle, double rollerVoltage, double centraliserVoltage){
        this.intakeAngle = intakeAngle;
        this.rollerVoltage = rollerVoltage;
        this.centraliserVoltage = centraliserVoltage;
    }
}
