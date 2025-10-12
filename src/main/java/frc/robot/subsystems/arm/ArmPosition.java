package frc.robot.subsystems.arm;

public enum ArmPosition { // todo
    PRE_CORAL_SCORE(1.084, true),

    TILTED_BRANCH_CORAL_SCORE(0.421, true),
    TILTED_BRANCH_CORAL_SCORE_POST(1.841, true),

    L4(0.36, true), //0.51
    L4_POST(1.841, true),

    INTAKE_ALGAE(0, false),

    NET(1.193, false),
    CLIMB(0, false),

    DOWNWARDS(1.5 * Math.PI, false),

    UPWARDS(Math.PI / 2, false),

    PROCESSOR(-0.223, false);

    private final double angle;
    public final boolean mirrorable;

    ArmPosition(double angle, boolean mirrorable) {
        this.mirrorable = mirrorable;
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
