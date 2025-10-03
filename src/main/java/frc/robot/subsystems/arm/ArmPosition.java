package frc.robot.subsystems.arm;

public enum ArmPosition {
    PRE_L2(0.345),
    L2(0.04),
    L2_FOLLOWTHROUGH(-0.4),

    PRE_L3(0.8964),
    L3(0.4171),
    L3_FOLLOWTHROUGH(-0.216),

    PRE_L4(0.9826738),
    L4(-0.05),
    L4_FOLLOWTHROUGH(0),

    INTAKE_ALGAE(0),

    PRE_NET(0),
    SCORE_NET(0),
    POST_NET(0),

    DOWNWARDS(4.71),
    DEFAULT_WITH_GAME_PIECE(Math.PI / 2),

    PROCESSOR(0),

    HANDOFF(4.653),

    EJECT_GAME_PIECE(0),

    INTAKE(1.5 * Math.PI);

    private final double angle;

    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
