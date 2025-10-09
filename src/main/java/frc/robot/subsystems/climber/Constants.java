package frc.robot.subsystems.climber;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;

public class Constants {

    public static final int MOTOR1_ID = 60;
    public static final int MOTOR2_ID = 61;
    public static final int ROLLER_MOTOR_ID = 62;

    public static final double ARM_POSITION_CONVERSION_FACTOR = 0.0051282;

    public static final int MAX_VELOCITY_LIMIT = 2;
    public static final int MIN_VELOCITY_LIMIT = -2;

    public static final SoftLimit VELOCITY_SOFTLIMIT = new SoftLimit(
            () -> MIN_VELOCITY_LIMIT,
            () -> MAX_VELOCITY_LIMIT
    );

    public static final Gains GAINS = new Gains(0, 0, 0);
    public static final Mass MASS = new Mass(() -> 0, () -> 0, 0);

    public static final double RETRACT_ANGLE = Math.PI / 2;
    public static final double TOLERANCE = 0;
    public static final double INITIAL_START_ANGLE = 0;
    public static final double OPEN_ANGLE = 0;


}
