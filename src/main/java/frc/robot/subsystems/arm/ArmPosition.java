package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public enum ArmPosition {
    L2(-5.3),
    L2_FOLLOWTHROUGH(0),
    L3(5),
    L3_FOLLOWTHROUGH(0),
    L4(0),
    L4_FOLLOWTHROUGH(0),
    ALGAE2(0),
    ALGAE3(0),
    NET(0),
    DEFAULT_WITHOUT_GAME_PIECE(-Math.PI/2),
    DEFAULT_WITH_GAME_PIECE(Math.PI/2),
    PROCESSOR(0),
    HANDOFF(-Math.PI/2),
    EJECT_GAME_PIECE(0),
    CHECK1(5.2),
    CHECK2(Math.PI * 2),
    CHECK3(Math.PI),
    INTAKE(0);

    private final double angle;

    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
