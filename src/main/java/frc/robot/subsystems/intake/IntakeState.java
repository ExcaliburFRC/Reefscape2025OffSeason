package frc.robot.subsystems.intake;

public enum IntakeState {
    FLOOR_INTAKE(3.08,-0.5,0),
    CENTERLIZE(2,0,0.4),
    DEFAULT(0.8635,-0.1,0.4),
//    DEFAULT(0.8635,-0.1,0.1),
    PRE_HANDOFF(0.8635,0,0),
//    PRE_HANDOFF(0.8635,0,0.3),
    HANDOFF(0.8635,0.8,0),
    L1_SCORE_PRE(1.352, 0, 0.1),
    L1_SCORE(1.352, 0.1,0.15),
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
