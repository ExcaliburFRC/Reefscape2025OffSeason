# Excalib Library

Excalib is a comprehensive library for FRC robot control, featuring motor control, swerve drive, SLAM, and utility functions. It now includes full integration with AdvantageKit for advanced logging and replay capabilities.

## AdvantageKit Integration

Excalib has been integrated with [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit), a logging and replay framework for FRC robots. This integration provides:

- **Hardware abstraction** via the IO pattern for easy simulation and testing
- **Automatic data logging** of motor telemetry (position, velocity, current, voltage, temperature)
- **Replay support** for debugging and analysis without physical hardware
- **Type-safe logging** using AdvantageKit's `@AutoLog` annotation

### Hardware IO Patterns

All hardware components in excalib now support the AdvantageKit IO pattern:

#### Motor Controllers

#### TalonFX Motors

```java
import frc.excalib.control.motor.controllers.TalonFXMotorIO;
import frc.excalib.control.motor.controllers.TalonFXMotorIOReal;
import org.littletonrobotics.junction.Logger;

// Create IO layer
TalonFXMotorIO motorIO = new TalonFXMotorIOReal(1); // CAN ID 1
TalonFXMotorIO.TalonFXMotorInputsAutoLogged inputs = new TalonFXMotorIO.TalonFXMotorInputsAutoLogged();

// In periodic method
motorIO.updateInputs(inputs);
Logger.processInputs("Motor/Left", inputs);

// Control the motor
motorIO.setVoltage(12.0);
```

#### SparkMax Motors

```java
import frc.excalib.control.motor.controllers.SparkMaxMotorIO;
import frc.excalib.control.motor.controllers.SparkMaxMotorIOReal;
import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.Logger;

// Create IO layer
SparkMaxMotorIO motorIO = new SparkMaxMotorIOReal(2, SparkMax.MotorType.kBrushless);
SparkMaxMotorIO.SparkMaxMotorInputsAutoLogged inputs = new SparkMaxMotorIO.SparkMaxMotorInputsAutoLogged();

// In periodic method
motorIO.updateInputs(inputs);
Logger.processInputs("Motor/Intake", inputs);

// Control the motor
motorIO.setPercentage(0.5);
```

#### SparkFlex Motors

```java
import frc.excalib.control.motor.controllers.FlexMotorIO;
import frc.excalib.control.motor.controllers.FlexMotorIOReal;
import com.revrobotics.spark.SparkFlex;
import org.littletonrobotics.junction.Logger;

// Create IO layer
FlexMotorIO motorIO = new FlexMotorIOReal(3, SparkFlex.MotorType.kBrushless);
FlexMotorIO.FlexMotorInputsAutoLogged inputs = new FlexMotorIO.FlexMotorInputsAutoLogged();

// In periodic method
motorIO.updateInputs(inputs);
Logger.processInputs("Motor/Shooter", inputs);

// Control the motor
motorIO.setVoltage(10.0);
```

#### IMU/Gyroscope

```java
import frc.excalib.control.imu.IMUIO;
import frc.excalib.control.imu.PigeonIOReal;
import frc.excalib.control.imu.NavXIOReal;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

// Create IO layer for Pigeon2
IMUIO gyroIO = new PigeonIOReal(5, new Rotation3d()); // CAN ID 5
IMUIO.IMUInputsAutoLogged inputs = new IMUIO.IMUInputsAutoLogged();

// Or for NavX
IMUIO gyroIO = new NavXIOReal(new Rotation3d());

// In periodic method
gyroIO.updateInputs(inputs);
Logger.processInputs("Gyro", inputs);

// Use the gyro
gyroIO.reset();
```

### Logged Data

**Motor IO classes** log the following data automatically:
- **Position** (rotations)
- **Velocity** (rotations per second)
- **Applied Voltage** (volts)
- **Current Draw** (amps)
- **Temperature** (celsius)

**IMU IO classes** log the following data automatically:
- **Yaw** (degrees)
- **Pitch** (degrees)
- **Roll** (degrees)
- **Acceleration X, Y, Z** (g-forces)
- **Connection Status** (boolean)

### Simulation and Testing

For simulation, unit testing, or replay, use the base IO classes (without "Real" suffix):

```java
// For simulation/testing/replay - no actual hardware
TalonFXMotorIO motorIO = new TalonFXMotorIO();
IMUIO gyroIO = new IMUIO();
```

The base IO classes provide default no-op implementations, perfect for:
- Unit testing
- Simulation mode
- **Log replay** - Test code changes against recorded robot logs
- Development without hardware

### Complete Example Subsystems

The `examples/` directory contains **complete, production-ready** subsystem implementations (no placeholder code):

#### Simple Examples
- **ExampleMotorSubsystem.java** - Basic single-motor subsystem with IO pattern
- **ExampleFlyWheelSubsystem.java** - FlyWheel with feedforward velocity control

#### Advanced Examples  
- **ExampleDriveSubsystem.java** - Tank/arcade drive with motors and IMU
- **ExampleArmSubsystem.java** - Arm with full PID + feedforward + gravity compensation
- **ExampleTurretSubsystem.java** - Turret with profiled PID and continuous angle wrapping

Each example includes:
- ✅ Complete, working control algorithms (PID, feedforward, motion profiling)
- ✅ Factory methods for real hardware vs. simulation/replay
- ✅ Comprehensive periodic logging with Logger.processInputs()
- ✅ Command factories for common operations
- ✅ Telemetry access methods
- ✅ Safety features (soft limits, voltage clamping)
- ✅ **No missing code** - ready to copy and use

### Accessing Underlying Motor Controllers

If you need access to advanced motor configuration methods, use the `getMotor()` method:

```java
TalonFXMotorIOReal motorIO = new TalonFXMotorIOReal(1);
TalonFXMotor motor = motorIO.getMotor();

// Now you can access all TalonFXMotor methods
motor.setIdleState(IdleState.BRAKE);
motor.setCurrentLimit(40, 60);
```

## Library Structure

- **control/** - Motor controllers, IMU interfaces, control loops
  - **motor/controllers/** - TalonFX, SparkMax, SparkFlex with AdvantageKit IO
  - **imu/** - Pigeon, NavX gyroscope interfaces
  - **gains/** - PID gains and SysId configuration
- **swerve/** - Swerve drive implementation
- **slam/** - SLAM and pose estimation
- **commands/** - Custom command patterns
- **additional_utilities/** - LED control, alliance utilities, and more

## Requirements

- **WPILib 2025.3.2** or later
- **AdvantageKit 4.1.2** (automatically included via vendordep)
- **Java 17** or later

## License

This library is part of the Reefscape2025OffSeason project.
