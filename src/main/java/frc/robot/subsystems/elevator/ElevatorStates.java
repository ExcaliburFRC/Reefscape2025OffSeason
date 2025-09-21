package frc.robot.subsystems.elevator;

public enum ElevatorStates {
    L2(0),
    L2_FOLLOWTHROUGH(0),
    L3(1.2),
    L3_FOLLOWTHROUGH(0),
    L4(0),
    L4_FOLLOWTHROUGH(0),
    ALGAE2(0),
    ALGAE3(0),
    HANDOFF(0.89),
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
