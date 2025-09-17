package frc.robot.subsystems.arm;

import frc.excalib.control.limits.SoftLimit;

public class Constants {
    public static final int FIRST_MOTOR_ID = 20;
    public static final int CAN_CODER_ID = 22;
    public static final SoftLimit VELOCITY_LIMIT = new SoftLimit(() -> -1, () -> 1);
    public static final double TOLERANCE = 0; // radians
    public static final double RPS_TO_RAD_PER_SEC = Math.PI * 2;
    public static final double ARM_COLISION_ELEVATOR_LENGTH = 0;

    public static final double ROTATIONS_TO_RAD = 2 * Math.PI;


}
