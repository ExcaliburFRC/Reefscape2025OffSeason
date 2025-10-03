package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import static java.lang.Math.*;

public class constants {
    public static int RIGHT_MOTOR_ID = 50;
    public static int LEFT_MOTOR_ID = 51;

    public static DoubleSupplier ELEVATOR_ANGLE = () -> PI / 2;

    public static double MAX_VELOCITY = 2;
    public static double MAX_ACCELERATION = 3.1;

    public static double TOLERANCE = 0.003;

    public static final double MAX_ELEVATOR_HIGHT = 1.41;

    public static final double POSITION_CONVERSION_FACTOR = 0.4756373706966272*0.0347879;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;


}
