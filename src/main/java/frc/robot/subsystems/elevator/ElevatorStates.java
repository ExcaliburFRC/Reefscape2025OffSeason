package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0.447),
    L2_POST(0.43),

    L3(0.76),
    L3_POST(0.7),

    L4(1.41),
    L4_POST(1.1),

    ALGAE2(0),
    ALGAE3(0),

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
