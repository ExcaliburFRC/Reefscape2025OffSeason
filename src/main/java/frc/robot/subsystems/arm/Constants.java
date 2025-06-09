package frc.robot.subsystems.arm;

import frc.excalib.control.limits.SoftLimit;

public class Constants {
    public static final int ANGLE_MOTOR_ID = 0;
    public static final int CAN_CODER_ID = 0;
    public static final SoftLimit VELOCITY_LIMIT = new SoftLimit(()-> 0, ()-> 30);
    public static final double TOLERANCE = 0.1;
    public static final double POSITION_CONVERSION_FACTOR = 0;
    public static final double RPS_TO_RAD_PER_SEC = 0;
    public static final double MIN_ELEVATOR_HIGHT_WITH_ARM = 0;
    public static final double ARM_LIMIT_CLOSE_ELEVATOR = 90;
    public static final double DEFAULT_MIN_ARM_LIMIT = 0;
    public static final double DEFAULT_MAX_ARM_LIMIT = 270;//TODO!!!
    public static final double ARM_LENGTH = 10;
    public static final double SOFTLIMIT_BUFFER = 10;

    public static final double ROTATIONS_TO_RAD = 2 *Math.PI;


}
