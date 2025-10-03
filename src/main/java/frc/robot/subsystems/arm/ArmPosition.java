package frc.robot.subsystems.arm;

public enum ArmPosition { // todo
    PRE_CORAL_SCORE(1.084),

    TILTED_BRANCH_CORAL_SCORE(0.421),
    TILTED_BRANCH_CORAL_SCORE_POST(1.841),

    L4(0.5614),
    L4_POST(1.841),

    INTAKE_ALGAE(0),

    NET(0),

    DOWNWARDS(1.5 * Math.PI),

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
