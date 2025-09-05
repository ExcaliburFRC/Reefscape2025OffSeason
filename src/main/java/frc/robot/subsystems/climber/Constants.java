package frc.robot.subsystems.climber;

import frc.excalib.control.gains.Gains;
import frc.excalib.control.limits.SoftLimit;
import frc.excalib.control.math.physics.Mass;

public class Constants {
    public static final int MOTOR1_ID = 50; // TODO: add values.
    public static final int MOTOR2_ID = 51; // TODO: add values.

    public static final double ARM_POSITION_CONVERSION_FACTOR = 0;

    public static final int MIN_VELOCITY_LIMIT = 0; // TODO: add values.
    public static final int MAX_VELOCITY_LIMIT = 0; // TODO: add values.

    public static final SoftLimit VELOCITY_SOFTLIMIT = new SoftLimit(
            () -> MIN_VELOCITY_LIMIT,
            () -> MAX_VELOCITY_LIMIT
    );

    public static final Gains GAINS = new Gains(0, 0, 0); // TODO: add values.
    public static final Mass MASS = new Mass(() -> 0, () -> 0, 0); // TODO: add values.

    public static final double RETRACT_ANGLE = Math.PI / 2;
    public static final double TOLERANCE = 0;
    public static final double INITIAL_START_ANGLE = 0;


}
