package frc.robot.subsystems.arm;

public enum ArmPosition {
    L2(0),
    L2_FOLLOWTHROUGH(0),
    L3(0),
    L3_FOLLOWTHROUGH(0),
    L4(40),
    L4_FOLLOWTHROUGH(40),
    ALGAE2(0),
    ALGAE3(0),
    NET(0),
    DEFAULT(0),
    PROCESSOR(0),
    HANDOFF(0),
    INTAKE(0);
    private  final double angle;
    private ArmPosition(double angle) {
        this.angle = angle;
    }
    public double getAngle() {
        return angle;
    }
}
