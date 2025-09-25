package frc.robot.subsystems.arm;

import frc.excalib.control.limits.SoftLimit;

public class Constants {
    public static final int FIRST_MOTOR_ID = 20;
    public static final int CAN_CODER_ID = 22;
    public static final SoftLimit VELOCITY_LIMIT = new SoftLimit(() -> -9, () -> 9);
    public static final double RPS_TO_RAD_PER_SEC = Math.PI * 2;
    public static final double ARM_COLISION_ELEVATOR_LENGTH = 0;

    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;

    public static final double INTAKE_HEIGHT = 0.32;
    public static final double ARM_LENGTH = 0.56;
}
