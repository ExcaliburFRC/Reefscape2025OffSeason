package frc.robot.subsystems.gripper;

public enum GripperStates {
    INTAKE_ALGAE(-0.8),
    INTAKE_CORAL(-0.7),
    ALGAE(-0.2),
    CORAL(0),
    HANDOFF(0),
    RELEASE_CORAL(0.8),
    RELEASE_ALGAE(0),
    VACENT(0);

    final double output;

    GripperStates(double output){
        this.output = output;
    }
}
