package frc.robot.subsystems.intake;

public enum IntakeState {
    FLOOR_INTAKE(3.08,-0.7,0),
    CENTERLIZE(2,0,0),
    DEFAULT(0.8635,-0.1,1),
    PRE_HANDOFF(0.8635,0,0.3),
    HANDOFF(2.5,0,0),
    L1_SCORE_PRE(0, 0, 0),
    L1_SCORE(0, 0, 0),
    GET_CORAL_BACK_FROM_ARM(0, 0, 0),
    EJECT_CORAL(2, 0, 0);
    final double intakeAngle;
    final double rollerVoltage;
    final double centraliserVoltage;
    IntakeState(double intakeAngle, double rollerVoltage, double centraliserVoltage){
        this.intakeAngle = intakeAngle;
        this.rollerVoltage = rollerVoltage;
        this.centraliserVoltage = centraliserVoltage;
    }
}
