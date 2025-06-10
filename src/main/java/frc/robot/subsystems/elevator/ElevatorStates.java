package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0),
    L2_FOLLOWTHROUGH(0),
    L3(0),
    L3_FOLLOWTHROUGH(0),
    L4(0),
    L4_FOLLOWTHROUGH(0),
    ALGAE2(0),
    ALGAE3(0),
    HANDOFF(0),
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
