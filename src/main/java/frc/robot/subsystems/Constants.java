package frc.robot.subsystems;

import frc.excalib.control.limits.SoftLimit;

public class Constants {
    public static final int ANGLE_MOTOR_ID = 0;
    public static final int CAN_CODER_ID = 0;
    public static final SoftLimit VELOCITY_LIMIT = new SoftLimit(()-> 0, ()-> 30);
    public static final double TOLERANCE = 0.1;
}
