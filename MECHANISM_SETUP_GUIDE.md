# Mechanism Setup Guide - Easy Integration with AdvantageKit

This guide shows how to easily set up excalib mechanisms with AdvantageKit integration using the provided helper classes.

## Quick Setup with MechanismSetupHelper

The `MechanismSetupHelper` class provides factory methods with sensible defaults for all excalib mechanisms.

### FlyWheel Setup

```java
import frc.excalib.mechanisms.MechanismSetupHelper;
import frc.excalib.control.motor.controllers.TalonFXMotor;

// Option 1: With default gains (recommended for testing)
Motor flywheelMotor = new TalonFXMotor(1);
FlyWheel flywheel = MechanismSetupHelper.createFlyWheel(flywheelMotor);

// Option 2: With custom gains
Gains customGains = new Gains(0.8, 0.0, 0.0, 0.15, 0.13, 0.01, 0.0);
FlyWheel flywheel = MechanismSetupHelper.createFlyWheel(flywheelMotor, customGains);
```

**Default Gains:**
- kP = 0.5, kI = 0.0, kD = 0.0
- kS = 0.1 (static friction), kV = 0.12 (velocity FF), kA = 0.01 (acceleration FF)

### Turret Setup

```java
import frc.excalib.mechanisms.MechanismSetupHelper;

// Option 1: With default gains, no limits (360° rotation)
Motor turretMotor = new TalonFXMotor(2);
DoubleSupplier angleSupplier = () -> turretMotor.getMotorPosition();
Turret turret = MechanismSetupHelper.createTurret(turretMotor, angleSupplier);

// Option 2: With soft limits (e.g., -90° to +90°)
Turret turret = MechanismSetupHelper.createTurretWithLimits(
    turretMotor,
    angleSupplier,
    MechanismSetupHelper.DEFAULT_TURRET_GAINS,
    -Math.PI / 2,  // minAngleRad
    Math.PI / 2    // maxAngleRad
);
```

**Default Gains:**
- kP = 8.0, kI = 0.0, kD = 0.2
- kS = 0.1 (static friction), kV = 0.5 (velocity FF), kA = 0.05 (acceleration FF)

### Arm Setup

```java
import frc.excalib.mechanisms.MechanismSetupHelper;

// Option 1: With default gains
Motor armMotor = new TalonFXMotor(3);
DoubleSupplier angleSupplier = () -> armMotor.getMotorPosition();
double armMassKg = 5.0; // Weight of your arm
Arm arm = MechanismSetupHelper.createArm(armMotor, angleSupplier, armMassKg);

// Option 2: With custom gains
Gains customArmGains = new Gains(6.0, 0.0, 0.15, 0.1, 1.8, 0.06, 0.6);
Arm arm = MechanismSetupHelper.createArm(armMotor, angleSupplier, armMassKg, customArmGains);
```

**Default Gains:**
- kP = 5.0, kI = 0.0, kD = 0.1
- kS = 0.1, kV = 1.5, kA = 0.05, **kG = 0.5** (gravity compensation)

### Linear Extension (Elevator) Setup

```java
import frc.excalib.mechanisms.MechanismSetupHelper;

// With default gains
Motor elevatorMotor = new TalonFXMotor(4);
DoubleSupplier positionSupplier = () -> elevatorMotor.getMotorPosition();
DoubleSupplier angleSupplier = () -> 0.0; // 0 if vertical, angle if angled
LinearExtension elevator = MechanismSetupHelper.createLinearExtension(
    elevatorMotor,
    positionSupplier,
    angleSupplier
);
```

**Default Gains:**
- kP = 4.0, kI = 0.0, kD = 0.1
- kS = 0.2, kV = 1.0, kA = 0.05, **kG = 0.3** (gravity compensation)

## Integrating with AdvantageKit

### Step 1: Create Motor IO Layer

```java
import frc.excalib.control.motor.controllers.*;

// For real hardware
TalonFXMotorIO motorIO = new TalonFXMotorIOReal(1);
TalonFXMotorInputsAutoLogged inputs = new TalonFXMotorInputsAutoLogged();

// For simulation
TalonFXMotorIO motorIO = new TalonFXMotorIO();
```

### Step 2: Create Mechanism

```java
// Get the underlying motor from IO layer
TalonFXMotorIOReal motorIOReal = (TalonFXMotorIOReal) motorIO;
Motor motor = motorIOReal.getMotor();

// Create mechanism using helper
FlyWheel flywheel = MechanismSetupHelper.createFlyWheel(motor);
```

### Step 3: Create Subsystem

```java
public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFXMotorIO motorIO;
    private final TalonFXMotorInputsAutoLogged inputs;
    private final FlyWheel mechanism;
    
    public FlywheelSubsystem(TalonFXMotorIO motorIO, FlyWheel mechanism) {
        this.motorIO = motorIO;
        this.inputs = new TalonFXMotorInputsAutoLogged();
        this.mechanism = mechanism;
    }
    
    @Override
    public void periodic() {
        motorIO.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }
    
    public Command runVelocity(DoubleSupplier velocityRPS) {
        return mechanism.smartVelocityCommand(velocityRPS, this);
    }
}
```

## Swerve Drive Setup

### Option 1: Using Existing Swerve

If you already have a configured `Swerve` subsystem:

```java
import frc.excalib.swerve.*;
import frc.excalib.examples.ExampleSwerveSubsystem;

// Wrap your existing swerve modules
Swerve existingSwerve = /* your configured swerve */;
ModulesHolder modules = existingSwerve.modules;

// Create IO wrappers for each module
SwerveModuleIO[] moduleIOs = new SwerveModuleIO[] {
    new SwerveModuleIOReal(modules.frontLeft),
    new SwerveModuleIOReal(modules.frontRight),
    new SwerveModuleIOReal(modules.backLeft),
    new SwerveModuleIOReal(modules.backRight)
};

// Create gyro IO
IMUIO gyroIO = new PigeonIOReal(5, new Rotation3d());

// Create swerve subsystem with AdvantageKit
ExampleSwerveSubsystem swerve = new ExampleSwerveSubsystem(moduleIOs, gyroIO);
```

### Option 2: From Scratch

```java
// Use the factory method (requires pre-configured modules)
ExampleSwerveSubsystem swerve = ExampleSwerveSubsystem.createReal(
    frontLeftModule,
    frontRightModule,
    backLeftModule,
    backRightModule,
    5  // Gyro CAN ID
);

// Or for simulation
ExampleSwerveSubsystem swerve = ExampleSwerveSubsystem.createSim();
```

## Complete Example: Robot Container

```java
public class RobotContainer {
    // Subsystems
    private final FlywheelSubsystem flywheel;
    private final ArmSubsystem arm;
    private final ExampleSwerveSubsystem swerve;
    
    public RobotContainer() {
        // Initialize based on robot mode
        if (RobotBase.isReal()) {
            flywheel = createRealFlywheel();
            arm = createRealArm();
            swerve = ExampleSwerveSubsystem.createSim(); // Replace with real modules
        } else {
            flywheel = createSimFlywheel();
            arm = createSimArm();
            swerve = ExampleSwerveSubsystem.createSim();
        }
        
        configureBindings();
    }
    
    private FlywheelSubsystem createRealFlywheel() {
        TalonFXMotorIO motorIO = new TalonFXMotorIOReal(1);
        TalonFXMotorIOReal motorIOReal = (TalonFXMotorIOReal) motorIO;
        FlyWheel mechanism = MechanismSetupHelper.createFlyWheel(motorIOReal.getMotor());
        return new FlywheelSubsystem(motorIO, mechanism);
    }
    
    private FlywheelSubsystem createSimFlywheel() {
        return new FlywheelSubsystem(new TalonFXMotorIO(), null);
    }
    
    private ArmSubsystem createRealArm() {
        TalonFXMotorIO motorIO = new TalonFXMotorIOReal(3);
        TalonFXMotorIOReal motorIOReal = (TalonFXMotorIOReal) motorIO;
        Motor motor = motorIOReal.getMotor();
        DoubleSupplier angleSupplier = () -> motor.getMotorPosition();
        Arm mechanism = MechanismSetupHelper.createArm(motor, angleSupplier, 5.0);
        return new ArmSubsystem(motorIO, mechanism);
    }
    
    private ArmSubsystem createSimArm() {
        return new ArmSubsystem(new TalonFXMotorIO(), null);
    }
    
    private void configureBindings() {
        // Flywheel on button A
        controller.a().whileTrue(flywheel.runVelocity(() -> 50.0));
        
        // Arm positions on face buttons
        controller.y().whileTrue(arm.moveToAngle(() -> Math.PI / 4));
        controller.x().whileTrue(arm.moveToAngle(() -> 0.0));
        
        // Swerve default command
        swerve.setDefaultCommand(
            swerve.driveFieldCentric(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
            )
        );
    }
}
```

## Tuning Your Mechanisms

### Finding kP, kI, kD

1. Start with defaults from `MechanismSetupHelper`
2. Increase kP until oscillation appears
3. Back off kP by 30-50%
4. Add kD if still oscillating (typically 10-20% of kP)
5. Only add kI if steady-state error persists (start very small, like 0.001)

### Finding Feedforward Values

**kS (Static Friction):**
- Slowly increase voltage until mechanism just starts moving
- That voltage is your kS value

**kV (Velocity Feedforward):**
- Command a constant velocity
- Measure actual voltage needed to maintain it
- kV = voltage / velocity

**kA (Acceleration Feedforward):**
- Usually 5-10% of kV
- Increase if mechanism struggles during rapid acceleration

**kG (Gravity Compensation):**
- For arms: voltage needed to hold horizontal position
- For elevators: voltage needed to hold at mid-height
- Typically 0.3-0.8 V depending on mass

## Common Issues

### Mechanism Oscillates
- **Too much kP**: Reduce by 20-30%
- **Not enough kD**: Add kD = 10-20% of kP

### Mechanism Drifts
- **Need kI**: Start with very small value (0.001)
- **Check mechanical friction**: May need higher kS

### Slow Response
- **Too little kP**: Increase gradually
- **Need more feedforward**: Tune kV and kA

### Mechanism Moves Opposite Direction
- **Inverted motor**: Add `motor.setInverted(REVERSE)`
- **Inverted sensor**: Check position supplier sign

## Best Practices

1. **Always test in simulation first** before running on real hardware
2. **Start with low gains** and increase gradually
3. **Use soft limits** to protect hardware
4. **Log everything** with AdvantageKit for debugging
5. **Create factory methods** for easy real vs. sim switching
6. **Tune one gain at a time** - don't change multiple values simultaneously
7. **Save working gains** as constants in your code

## See Also

- `EXAMPLE_SUBSYSTEMS_USAGE.md` - How to run the integrated examples
- `CONTROLLER_LAYOUT.md` - Controller bindings reference
- `src/main/java/frc/excalib/README.md` - excalib library documentation
- `src/main/java/frc/excalib/examples/` - Complete example implementations
