# AdvantageKit Integration Summary

This document summarizes the integration of AdvantageKit into the excalib library for the Reefscape2025OffSeason project.

## What is AdvantageKit?

AdvantageKit is a logging and replay framework for FRC robots that provides:
- Comprehensive data logging for analysis and debugging
- Hardware abstraction via the IO pattern
- Log replay for testing without physical hardware
- Automatic code generation for type-safe logging

Official repository: https://github.com/Mechanical-Advantage/AdvantageKit

## Integration Changes

### 1. AdvantageKit Vendordep

**File Added:** `vendordeps/AdvantageKit.json`

- **Version:** 4.1.2
- **FRC Year:** 2025
- **Maven Repository:** https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release/

This file was previously referenced in `build.gradle` (line 93) but was missing, causing build failures. The vendordep includes:
- Java dependencies: `akit-java` v4.1.2
- JNI dependencies: `akit-wpilibio` v4.1.2 (for all platforms)

### 2. Motor Controller IO Patterns

Created AdvantageKit IO patterns for all motor controllers in excalib:

#### TalonFX Motor
- **TalonFXMotorIO.java** - Base IO interface with `@AutoLog` annotation
- **TalonFXMotorIOReal.java** - Real hardware implementation using `TalonFXMotor`

#### SparkMax Motor
- **SparkMaxMotorIO.java** - Base IO interface with `@AutoLog` annotation
- **SparkMaxMotorIOReal.java** - Real hardware implementation using `SparkMaxMotor`

#### SparkFlex Motor
- **FlexMotorIO.java** - Base IO interface with `@AutoLog` annotation
- **FlexMotorIOReal.java** - Real hardware implementation using `FlexMotor`

### 3. Logged Motor Telemetry

Each motor IO class automatically logs the following data:
- **Position** (rotations) - `positionRotations`
- **Velocity** (rotations per second) - `velocityRotationsPerSecond`
- **Applied Voltage** (volts) - `appliedVolts`
- **Current Draw** (amps) - `currentAmps`
- **Temperature** (celsius) - `temperatureCelsius`

The `@AutoLog` annotation generates the necessary boilerplate code for logging these inputs.

### 4. Documentation

**File Added:** `src/main/java/frc/excalib/README.md`

Comprehensive documentation including:
- Overview of AdvantageKit integration
- Usage examples for each motor controller type
- Simulation and testing guidance
- Library structure
- Requirements

**File Added:** `src/main/java/frc/excalib/examples/ExampleMotorSubsystem.java`

Example subsystem demonstrating:
- Proper IO pattern usage in a subsystem
- Factory methods for real hardware vs. simulation
- Periodic logging with `Logger.processInputs()`
- Accessing motor telemetry from logged inputs

## Usage Example

### Before (without AdvantageKit IO pattern):

```java
public class MySubsystem extends SubsystemBase {
    private final TalonFXMotor motor;
    
    public MySubsystem() {
        motor = new TalonFXMotor(1);
    }
    
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
```

### After (with AdvantageKit IO pattern):

```java
public class MySubsystem extends SubsystemBase {
    private final TalonFXMotorIO motorIO;
    private final TalonFXMotorInputsAutoLogged inputs;
    
    public MySubsystem(TalonFXMotorIO motorIO) {
        this.motorIO = motorIO;
        this.inputs = new TalonFXMotorInputsAutoLogged();
    }
    
    // Factory for real hardware
    public static MySubsystem createReal() {
        return new MySubsystem(new TalonFXMotorIOReal(1));
    }
    
    // Factory for simulation
    public static MySubsystem createSim() {
        return new MySubsystem(new TalonFXMotorIO());
    }
    
    @Override
    public void periodic() {
        motorIO.updateInputs(inputs);
        Logger.processInputs("MySubsystem/Motor", inputs);
    }
    
    public void setVoltage(double volts) {
        motorIO.setVoltage(volts);
    }
}
```

## Benefits

1. **Hardware Abstraction** - Easy to switch between real hardware and simulation
2. **Comprehensive Logging** - All motor telemetry automatically logged
3. **Replay Support** - Debug and analyze matches without physical robot
4. **Type Safety** - Compile-time checking of logged values
5. **Testing** - Write unit tests without hardware dependencies
6. **Debugging** - View all motor data in AdvantageScope

## Build Configuration

No changes needed to `build.gradle` - the annotation processor was already configured:

```groovy
def akitJson = new groovy.json.JsonSlurper().parseText(new File(projectDir.getAbsolutePath() + "/vendordeps/AdvantageKit.json").text)
annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"
```

## Next Steps

To use AdvantageKit in your subsystems:

1. Replace direct motor instantiation with IO pattern
2. Create factory methods for real vs. simulation
3. Call `updateInputs()` and `Logger.processInputs()` in `periodic()`
4. Access motor telemetry from the `inputs` object instead of motor directly
5. View logged data in AdvantageScope

## References

- AdvantageKit Documentation: https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/docs
- Excalib README: `src/main/java/frc/excalib/README.md`
- Example Subsystem: `src/main/java/frc/excalib/examples/ExampleMotorSubsystem.java`
- Test Subsystem (existing usage): `src/main/java/frc/robot/subsystems/test/Test.java`

## Version Information

- **AdvantageKit Version:** 4.1.2
- **FRC Year:** 2025
- **WPILib Version:** 2025.3.2
- **Java Version:** 17
