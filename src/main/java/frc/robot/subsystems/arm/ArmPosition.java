package frc.robot.subsystems.arm;

public enum ArmPosition {
    LEFT_PRE_L2(0.345),
    LEFT_L2(0.04),
    LEFT_L2_FOLLOWTHROUGH(-0.4),

    LEFT_PRE_L3(0.8964),
    LEFT_L3(0.4171),
    LEFT_L3_FOLLOWTHROUGH(-0.216),

    LEFT_PRE_L4(0.9826738),
    LEFT_L4(-0.05),
    LEFT_L4_FOLLOWTHROUGH(0),

    RIGHT_PRE_L2(-5.3),
    RIGHT_L2(-5.3),
    RIGHT_L2_FOLLOWTHROUGH(0),
    RIGHT_PRE_L3(5),
    RIGHT_L3(5),
    RIGHT_L3_FOLLOWTHROUGH(0),
    RIGHT_PRE_L4(0),
    RIGHT_L4(0),
    RIGHT_L4_FOLLOWTHROUGH(0),

    RIGHT_ALGAE2(0),
    RIGHT_ALGAE3(0),

    LEFT_ALGAE2(0),
    LEFT_ALGAE3(0),

    GO_THROUGH_FOR_LEFT(2),
    GO_THROUGH_FOR_RIGHT(-5.8),

    RIGHT_NET(0),
    RIGHT_NET_POST(0),
    LEFT_NET(0),
    LEFT_NET_POST(0),

    DEFAULT_WITHOUT_GAME_PIECE(-Math.PI / 2),
    DEFAULT_WITH_GAME_PIECE(Math.PI/2),

    LEFT_PROCESSOR(0),
    RIGHT_PROCESSOR(0),

    HANDOFF(-1.63),
    EJECT_GAME_PIECE(0),
    INTAKE(0);

    private final double angle;

    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
