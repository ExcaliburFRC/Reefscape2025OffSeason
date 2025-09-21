package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public enum ArmPosition {
    LEFT_PRE_L2(-5.3),
    LEFT_L2(-5.3),
    LEFT_L2_FOLLOWTHROUGH(0),
    LEFT_PRE_L3(5),
    LEFT_L3(5),
    LEFT_L3_FOLLOWTHROUGH(0),
    LEFT_PRE_L4(0),
    LEFT_L4(0),
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

    RIGHT_NET(0),
    RIGHT_NET_POST(0),
    LEFT_NET(0),
    LEFT_NET_POST(0),

    DEFAULT_WITHOUT_GAME_PIECE(-Math.PI / 2),
    DEFAULT_WITH_GAME_PIECE(1.5),

    LEFT_PROCESSOR(0),
    RIGHT_PROCESSOR(0),

    HANDOFF(-1.6),
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
