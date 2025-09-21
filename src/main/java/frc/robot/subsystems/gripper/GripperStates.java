package frc.robot.subsystems.gripper;

public enum GripperStates {
    INTAKE_ALGAE(0),
    INTAKE_CORAL(0),
    ALGAE(0),
    CORAL(0),
    HANDOFF(0),
    RELEASE_CORAL(0),
    RELEASE_ALGAE(0),
    VACENT(0);

    double output;

    GripperStates(double output){
        this.output = output;
    }
}
