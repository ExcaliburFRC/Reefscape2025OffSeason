package frc.robot.subsystems.arm;

public enum ArmPosition { // todo
    PRE_CORAL_SCORE(0.3),

    TILTED_BRANCH_CORAL_SCORE(0),
    TILTED_BRANCH_CORAL_SCORE_POST(0),

    L4(-0.05),
    L4_POST(0),

    INTAKE_ALGAE(0),

    PRE_NET(0),
    SCORE_NET(0),
    POST_NET(0),

    DOWNWARDS(4.71),

    UPWARDS(Math.PI / 2),

    PROCESSOR(0);

    private final double angle;

    ArmPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
