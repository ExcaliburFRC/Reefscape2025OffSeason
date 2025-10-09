package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0.455),
    L2_POST(0.16),

    L3(0.83),
    L3_POST(0.5),

    PRE_L4(1.46),
    L4(1.46),
    L4_POST(0.98),

    ALGAE2(0.67), // todo
    ALGAE3(1), // todo

    HANDOFF(0.95),

    NET(1.44),
    PROCESSOR(0.442),

    DEFAULT_WITH_CORAL(0.4),
    DEFAULT_WITH_ALGAE(0.6),
    SAFE_HEIGHT(1),
    L1_HEIGHT(1.10);

    private final double height;

    ElevatorStates(double height) {
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
