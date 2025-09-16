package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static java.lang.Math.*;

public class constants {
    public static int RIGHT_MOTOR_ID = 50;
    public static int LEFT_MOTOR_ID = 51;

    public static DoubleSupplier ELEVATOR_ANGLE = () -> PI / 2;

    public static double MAX_VELOCITY = 3;
    public static double MAX_ACCELERATION = 1;

    public static double TOLERANCE = 0.005;

    public static final double MAX_ELEVATOR_HIGHT = 0;
    public static final double MIN_ELEVATOR_HIGHT = 0;

    public static final double MIN_ELEVATOR_HIGHT_WITH_OPEN_ARM = 0;

    public static final double UPWARDS_ARM_MIN_LIMIT = 0;
    public static final double UPWARDS_ARM_MAX_LIMIT = 0;

    public static final double POSITION_CONVERSION_FACTOR = 0.4756373706966272*0.0347879;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;


}
