package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0.487),
    L2_FOLLOWTHROUGH(0.377),
    L3(0.787),
    L3_FOLLOWTHROUGH(0.757),
    L4(1.331),
    L4_FOLLOWTHROUGH(1.4),
    ALGAE2(0),
    ALGAE3(0),
    HANDOFF(0.92),
    PRE_HANDOFF(1),
    NET(0),
    PROCESSOR(0),
    DEFAULT_WITH_GAME_PIECE(0.4),
    DEFAULT_WITHOUT_GAME_PIECE(0);

    private final double height;
    private ElevatorStates(double height){
        this.height = height;
    }

    public double getHeight() {
        return height;
    }
}
