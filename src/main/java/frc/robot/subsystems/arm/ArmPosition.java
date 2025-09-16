package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public enum ArmPosition {
    L2(0),
    L2_FOLLOWTHROUGH(0),
    L3(0),
    L3_FOLLOWTHROUGH(0),
    L4(0),
    L4_FOLLOWTHROUGH(0),
    ALGAE2(0),
    ALGAE3(0),
    NET(0),
    DEFAULT_WITHOUT_GAME_PIECE(Math.PI),
    DEFAULT_WITH_GAME_PIECE(-Math.PI),
    PROCESSOR(0),
    HANDOFF(0),
    EJECT_GAME_PIECE(0),
    CHECK1(Math.PI/2),
    CHECK2(Units.degreesToRadians(30)),
    CHECK3(Math.PI),
    INTAKE(0);

    private  final double angle;
    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
