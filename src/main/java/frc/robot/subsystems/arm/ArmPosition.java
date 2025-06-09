package frc.robot.subsystems.arm;

public enum ArmPosition {
    L1(20),
    L2(0),
    L3(0),
    L4(40),
    DEFAULT(0),
    INTAKE(0);
    private  final double angle;
    private ArmPosition(double angle) {
        this.angle = angle;
    }
    public double getAngle() {
        return angle;
    }
}
