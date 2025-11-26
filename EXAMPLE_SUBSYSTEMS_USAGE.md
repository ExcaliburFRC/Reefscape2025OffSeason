# Using AdvantageKit Example Subsystems

This guide shows how to run the complete AdvantageKit-integrated example subsystems with a simulated controller.

## Quick Start

### Option 1: Run Examples Alongside Existing Code (Recommended)

Keep your existing robot code and add examples for testing:

1. **Open `Main.java`** and temporarily change the robot class:
```java
// Change this line:
RobotBase.startRobot(Robot::new);

// To this:
RobotBase.startRobot(RobotWithExamples::new);
```

2. **Connect a controller** (Xbox or PS5) to port 0 in the Driver Station

3. **Enable the robot** in simulation mode

4. **Test the examples** using the controller bindings below

### Option 2: Integrate Examples Into Existing RobotContainer

Add individual subsystems to your existing `RobotContainer.java`:

```java
import frc.excalib.examples.*;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer {
    // Add example subsystems
    private final ExampleArmSubsystem armExample;
    private final ExampleTurretSubsystem turretExample;
    
    public RobotContainer() {
        // Create subsystems based on robot mode
        if (RobotBase.isReal()) {
            armExample = ExampleArmSubsystem.createReal(10);
            turretExample = ExampleTurretSubsystem.createReal(11);
        } else {
            armExample = ExampleArmSubsystem.createSim();
            turretExample = ExampleTurretSubsystem.createSim();
        }
        
        // Configure bindings...
    }
}
```

## Controller Bindings (Xbox Controller on Port 0)

### Drive Subsystem
- **Left Stick**: Drive forward/backward and strafe left/right
- **Right Stick X**: Rotate robot
- **A Button**: Reset gyro heading to 0°
- **B Button**: Stop drive motors

### Arm Subsystem
- **Y Button**: Move arm to 45° (hold button)
- **X Button**: Move arm to 0° horizontal (hold button)
- **Left Bumper**: Move arm to 90° vertical (hold button)
- **Right Bumper + Right Trigger**: Manual arm control (trigger controls voltage)

### Turret Subsystem
- **D-Pad Up**: Rotate turret to 0°
- **D-Pad Right**: Rotate turret to 90°
- **D-Pad Down**: Rotate turret to 180°
- **D-Pad Left**: Rotate turret to -90°

### Flywheel Subsystem
- **Start Button**: Run flywheel at 50 RPS (hold button)
- **Back Button**: Stop flywheel

### Motor Subsystem
- **Left Trigger**: Run simple motor forward at 6V

## Features Demonstrated

### ✅ Complete AdvantageKit Integration
- All hardware uses IO pattern for abstraction
- Full telemetry logging (position, velocity, current, voltage, temperature)
- Replay support - record logs and replay without hardware

### ✅ Hardware Abstraction
- Automatic hardware vs. simulation selection
- Factory methods: `createReal()` and `createSim()`
- Works identically in simulation and on real robot

### ✅ Production-Ready Control
- **Arm**: PID + ArmFeedforward with gravity compensation
- **Turret**: Profiled PID with motion constraints and continuous wrapping
- **Flywheel**: Velocity control with feedforward
- **Drive**: Arcade and tank drive with IMU integration

### ✅ Safety Features
- Voltage limiting (±12V)
- Soft limits on turret rotation
- Position and velocity tolerances
- At-setpoint detection

## Viewing Logged Data

### Option 1: NetworkTables (Real-time)
1. Open **Shuffleboard** or **Glass**
2. Navigate to the NetworkTables view
3. Look for subsystem data under:
   - `Arm/` - Arm telemetry
   - `Turret/` - Turret telemetry
   - `Drive/` - Drive and IMU telemetry
   - `ExampleFlyWheel/` - Flywheel telemetry

### Option 2: AdvantageScope (Replay)
1. Run the robot in simulation to generate log files
2. Open **AdvantageScope**
3. Load the log file from the project directory or `/U/logs` (on real robot)
4. View all logged data with graphs and 3D visualization
5. Use replay mode to test code changes against recorded logs

## Testing Replay Functionality

1. **Record a log**:
   - Run the robot in simulation mode (`isReplay = false` in RobotWithExamples.java)
   - Enable the robot and move the controllers
   - A `.wpilog` file will be created

2. **Replay the log**:
   - Set `isReplay = true` in RobotWithExamples.java
   - Run the robot
   - Select the log file when prompted (or set the path in AdvantageScope)
   - The robot will replay all inputs and log outputs

3. **Test code changes**:
   - Make changes to subsystem control logic
   - Replay the same log
   - Compare outputs to see the effect of your changes

## Autonomous Mode

An example autonomous sequence is included that:
1. Moves the arm to 45°
2. Rotates the turret 360° (via 180° and back to 0°)
3. Runs the flywheel at 30 RPS for 3 seconds
4. Stops all mechanisms

Enable autonomous mode to see this in action!

## Customization

### Changing CAN IDs

Edit `RobotContainerWithExamples.java`, line 49-53:
```java
motorSubsystem = ExampleMotorSubsystem.createReal(1);      // CAN ID 1
flywheelSubsystem = ExampleFlyWheelSubsystem.createReal(2, ...); // CAN ID 2
driveSubsystem = ExampleDriveSubsystem.createReal(3, 4, 5); // CAN IDs 3, 4, 5
armSubsystem = ExampleArmSubsystem.createReal(6);          // CAN ID 6
turretSubsystem = ExampleTurretSubsystem.createReal(7);    // CAN ID 7
```

### Tuning Control Parameters

Each subsystem has tuning constants in its `createReal()` method:

**Arm** (`ExampleArmSubsystem.java`, lines 75-80):
```java
PIDController pid = new PIDController(5.0, 0.0, 0.1);
ArmFeedforward ff = new ArmFeedforward(0.1, 0.5, 1.5, 0.05);
```

**Turret** (`ExampleTurretSubsystem.java`, lines 77-82):
```java
ProfiledPIDController pid = new ProfiledPIDController(8.0, 0.0, 0.2, constraints);
```

**Flywheel** (`RobotContainerWithExamples.java`, lines 158-166):
```java
return new Gains(
    0.5,  // kp
    0.0,  // ki
    0.0,  // kd
    0.1,  // ks
    0.12, // kv
    0.01, // ka
    0.0   // kg
);
```

## Troubleshooting

### Controller Not Working
- Verify controller is connected to port 0 in Driver Station
- Check that the robot is enabled
- Try pressing the "A" button to reset the gyro

### Subsystems Not Moving in Simulation
- This is expected - simulation mode provides no physics
- The IO pattern logs all commands sent to motors
- Use AdvantageScope to view commanded voltages
- For physics simulation, integrate with WPILib's simulation framework

### Build Errors
- Ensure AdvantageKit.json vendordep is present in `vendordeps/`
- Run `./gradlew build` to verify compilation
- Check that all import statements resolve correctly

## Reverting to Original Code

To switch back to your original robot code:

1. **Edit `Main.java`**:
```java
// Change back to:
RobotBase.startRobot(Robot::new);
```

2. **Or delete/rename the example files**:
   - `RobotWithExamples.java`
   - `RobotContainerWithExamples.java`

Your original `Robot.java` and `RobotContainer.java` remain unchanged.

## Next Steps

1. **Study the example code** - See how IO patterns work
2. **Copy patterns to your subsystems** - Use examples as templates
3. **Test with replay** - Record logs and test code changes
4. **View in AdvantageScope** - Visualize all logged data

For more information, see:
- `src/main/java/frc/excalib/README.md` - excalib library documentation
- `ADVANTAGEKIT_INTEGRATION.md` - Integration details
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/docs)
