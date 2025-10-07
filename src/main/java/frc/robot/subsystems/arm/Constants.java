package frc.robot.subsystems.arm;

import frc.excalib.control.limits.SoftLimit;

public class Constants {
    public static final int FIRST_MOTOR_ID = 20;
    public static final int CAN_CODER_ID = 22;
    public static final SoftLimit VELOCITY_LIMIT = new SoftLimit(() -> -8, () -> 8);
    public static final double RPS_TO_RAD_PER_SEC = Math.PI * 2;

    public static final double INTAKE_HEIGHT = 0.32;
    public static final double ARM_LENGTH = 0.65;

    public static final double ARM_MASS_TO_AXIS_OFFSET = 0.07079632;
    public static final double MAX_SCORE_RAD = 0.88;
}
