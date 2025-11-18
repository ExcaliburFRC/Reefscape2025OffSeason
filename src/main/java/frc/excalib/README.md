# Excalib Library

Excalib is a comprehensive library for FRC robot control, featuring motor control, swerve drive, SLAM, and utility functions. It now includes full integration with AdvantageKit for advanced logging and replay capabilities.

## AdvantageKit Integration

Excalib has been integrated with [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit), a logging and replay framework for FRC robots. This integration provides:

- **Hardware abstraction** via the IO pattern for easy simulation and testing
- **Automatic data logging** of motor telemetry (position, velocity, current, voltage, temperature)
- **Replay support** for debugging and analysis without physical hardware
- **Type-safe logging** using AdvantageKit's `@AutoLog` annotation

### Motor Controller IO Pattern

All motor controllers in excalib now support the AdvantageKit IO pattern:

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

### Logged Data

Each motor IO class logs the following data automatically:

- **Position** (rotations)
- **Velocity** (rotations per second)
- **Applied Voltage** (volts)
- **Current Draw** (amps)
- **Temperature** (celsius)

### Simulation and Testing

For simulation or unit testing, use the base IO classes (without "Real" suffix):

```java
// For simulation/testing - no actual hardware
TalonFXMotorIO motorIO = new TalonFXMotorIO();
```

The base IO classes provide default no-op implementations, perfect for:
- Unit testing
- Simulation mode
- Log replay
- Development without hardware

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
