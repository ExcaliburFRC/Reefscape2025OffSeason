package frc.robot.subsystems;

public enum ElevatorStates {
    L2(0),
    L3(0),
    L4(0),
    SCORE(0),
    ALGAE_L2(0),
    ALGAE_L3(0),
    CORAL_INTAKE(0),
    NET(0),
    PROCESSOR(0),
    DEFAULT(0);

    private final double height;
    private ElevatorStates(double height){
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
