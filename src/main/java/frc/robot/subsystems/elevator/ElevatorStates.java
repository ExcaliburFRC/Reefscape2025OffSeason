package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0.455),
    L2_POST(0.16),

    L3(0.855),
    L3_POST(0.56),

    L4(1.46),
    L4_POST(1),

    ALGAE2(0), // todo
    ALGAE3(0), // todo

    HANDOFF(0.92),

    NET(0),
    PROCESSOR(0),

    DEFAULT_WITH_CORAL(0.4),
    DEFAULT_WITH_ALGAE(0.6),
    SAFE_HEIGHT(0.98);

    private final double height;

    private ElevatorStates(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
