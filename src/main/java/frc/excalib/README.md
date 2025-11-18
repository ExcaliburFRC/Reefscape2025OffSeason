# Excalib - High-Performance FRC Robot Library

Excalib is a comprehensive robotics library designed to give FRC teams a competitive edge through productivity-enhancing utilities, smart abstractions, and battle-tested patterns.

## 🚀 Core Features

### Motor Control & Mechanisms
- **Universal Motor Interface** - Unified API for TalonFX, SparkMax, and FlexMotor controllers
- **Motor Groups** - Synchronized multi-motor control with automatic follower configuration
- **Mechanism Abstractions** - Pre-built Arm, LinearExtension, FlyWheel, and Turret implementations
- **Smart Soft Limits** - Configurable soft limits with continuous wrapping support

### Command Utilities
- **CommandMutex** - Ensure only one command runs at a time with automatic cancellation
- **ContinuouslyConditionalCommand** - Commands that re-evaluate conditions continuously
- **MapCommand** - Transform and map command outputs dynamically
- **Command Decorators** - Add retry logic, timeouts, and rate limiting to any command

### Math & Control
- **Advanced Math Utils** - Geometry helpers, circle/line intersections, vector operations
- **EMA Filter** - Exponential moving average filter for signal smoothing
- **Periodic Scheduler** - Execute tasks at specific intervals with drift compensation
- **Custom Gains** - Structured PID + feedforward gain containers with SysId integration

### SLAM & Localization
- **PoseEstimator** - Enhanced pose estimation with vision integration
- **Odometry** - Swerve drive odometry with multi-sensor fusion
- **Aurora Client** - Integration with Aurora localization system

### Swerve Drive
- **Swerve** - Complete swerve drive implementation with field-centric control
- **SwerveModule** - Individual module control with optimal state selection
- **Acceleration Utilities** - Smooth acceleration limiting for better control

### Alliance & Field Utilities
- **AllianceUtils** - Automatic pose mirroring and alliance-aware transformations
- **AlliancePose** - Poses that automatically adjust based on driver station alliance

### Additional Utilities
- **PS5Controller** - Extended PS5 DualSense controller support
- **DoubleClickClient** - Detect double-click patterns on buttons
- **DoubleKeyMap** - Two-key lookup map for complex state management
- **LED Control** - Simple LED pattern and color management
- **IMU Abstraction** - Unified interface for Pigeon and NavX gyroscopes

---

## 🆕 New Productivity Features

### 1. Subsystem Builder (`subsystems/`)
**SUPER EASY** - Create subsystems with zero boilerplate:

```java
public class IntakeSubsystem extends SubsystemBase {
    private final SubsystemBuilder builder;
    
    public IntakeSubsystem() {
        builder = SubsystemBuilder.create(this, "intake")
            .withState("idle", motor::stopMotor)
            .withState("intaking", () -> motor.setPercentage(0.8))
            .withTrigger("has_piece", sensor::hasGamePiece)
            .withAutoTelemetry()
            .build();
    }
    
    public Command intakeCommand() {
        return builder.commandForState("intaking");
    }
}
```

**Benefits:** Write subsystems 10x faster, automatic telemetry, zero boilerplate

### 2. Quick Subsystem (`subsystems/`)
**ULTRA FAST** - Complete subsystem in 10 lines:

```java
public class IntakeSubsystem extends QuickSubsystem {
    public IntakeSubsystem(Motor motor, DigitalInput sensor) {
        super("intake");
        addState("idle", motor::stopMotor);
        addState("intaking", () -> motor.setPercentage(0.8));
        addSensor("has_piece", sensor::get);
        setDefaultState("idle");
    }
}
```

**Benefits:** Fastest possible subsystem creation, perfect for simple subsystems

### 3. Smart Motor (`subsystems/`)
Motors with superpowers - automatic telemetry, safety, PID:

```java
SmartMotor shooter = SmartMotor.wrap(shooterMotor, "shooter")
    .withCurrentLimit(60.0)
    .withTelemetry()
    .withSmartVelocityControl(gains)
    .build();

shooter.setTargetVelocity(3000); // Automatic PID + telemetry!
```

**Benefits:** No manual telemetry, built-in safety, smart control

### 4. State Machine (`subsystems/`)
Professional state machines for autonomous:

```java
StateMachine auto = StateMachine.builder()
    .state("drive", driveCommand)
        .onCondition(() -> atTarget(), "intake")
    .state("intake", intakeCommand)
        .onCondition(() -> hasGamePiece(), "score")
    .state("score", scoreCommand)
    .initialState("drive")
    .build();
```

**Benefits:** Clean autonomous code, automatic transitions, visual debugging

### 5. Command Factory (`subsystems/`)
Common command patterns, zero boilerplate:

```java
// Position control with one line
Command move = CommandFactory.positionControl(
    arm, () -> arm.setPosition(target), 
    arm::getPosition, target, 0.01
);

// Run until condition
Command intake = CommandFactory.runUntil(
    intake, intake::run, sensor::hasGamePiece
);

// Ramp smoothly
Command spinUp = CommandFactory.ramp(
    shooter, shooter::setSpeed, 0, 3000, 2.0
);
```

**Benefits:** No repetitive command code, standard patterns, fast development

### 6. Subsystem Health (`subsystems/`)
Automatic health monitoring and diagnostics:

```java
SubsystemHealth health = SubsystemHealth.monitor("shooter")
    .checkMotor("motor", motor::isConnected)
    .checkValue("velocity", motor::getVelocity, 0, 6000)
    .checkValue("temp", motor::getTemperature, 0, 80)
    .build();

health.update(); // In periodic
if (!health.isHealthy()) { /* handle errors */ }
```

**Benefits:** Catch problems before matches, automatic diagnostics, driver alerts

### 7. Auto Selector (`subsystems/`)
Simple autonomous selection:

```java
AutoSelector auto = AutoSelector.builder()
    .addAuto("Score 3", scoreThree())
    .addAuto("Score 2", scoreTwo())
    .addAuto("Mobility", mobility())
    .withDefault("Score 3")
    .build();

return auto.getSelected(); // In getAutonomousCommand()
```

**Benefits:** Dashboard selection, easy configuration, no code changes needed

### 8. Auto-Tuning Utilities (`control/autotuning/`)
Automatically tune PID controllers using characterization data:

```java
AutoTuner tuner = new AutoTuner(motor, feedbackSensor);
Gains optimizedGains = tuner.characterizeAndTune(
    AutoTuneMethod.ZIEGLER_NICHOLS,
    AutoTuneConfig.forVelocityControl()
);
motor.configPID(optimizedGains);
```

**Benefits:** Eliminates hours of manual PID tuning, ensures consistent performance

### 9. Telemetry Suite (`telemetry/`)
Enhanced logging with automatic performance tracking:

```java
TelemetryManager.getInstance()
    .recordLatency("vision/processing")
    .recordEvent("auto/milestone", "reached_scoring_position");
```

**Benefits:** Real-time performance monitoring, historical data analysis, competition debugging

### 10. Simulation Helpers (`simulation/`)
Mock objects for comprehensive unit testing:

```java
@Test
void testArmMotion() {
    SimulatedMotor motor = SimulatedMotor.create(DCMotor.kNEO);
    Arm arm = new Arm(motor, ...);
    
    arm.setPosition(Math.PI / 2);
    simulator.advance(Duration.ofSeconds(2));
    
    assertEquals(Math.PI / 2, arm.getPosition(), 0.01);
}
```

**Benefits:** Test code without hardware, catch bugs early, CI/CD integration

### 5. Command Decorators (`commands/decorators/`)
Add resilience and control flow to existing commands:

```java
Command robustIntake = intakeCommand
    .withRetry(3)
    .withTimeout(5.0)
    .withRateLimit(10) // max 10 Hz execution
    .withDeadband(0.02);
```

**Benefits:** More robust autonomous, easier error handling, prevents system overload

### 6. Motion Profiling (`control/motion/`)
Pre-configured motion profiles for smooth movement:

```java
TrapezoidProfile profile = TrapezoidProfile.builder()
    .withMaxVelocity(3.0)
    .withMaxAcceleration(2.0)
    .build();

Command moveToPosition = arm.followProfile(
    profile.calculate(currentPos, targetPos, dt)
);
```

**Benefits:** Smoother robot motion, reduced mechanical stress, better cycle times

### 7. Vision Pipeline Helpers (`vision/`)
Common vision processing patterns:

```java
VisionPipeline pipeline = VisionPipeline.builder()
    .withColorThreshold(ColorRange.RED)
    .withContourFilter(ContourFilters.minArea(100))
    .withTargetSelector(TargetSelectors.closest())
    .build();

Optional<VisionTarget> target = pipeline.process(cameraFrame);
```

**Benefits:** Faster vision development, proven filtering techniques, consistent results

### 8. Dashboard Widgets (`dashboard/`)
Auto-generate Shuffleboard layouts from code:

```java
@DashboardWidget(tab = "Drivetrain", type = WidgetType.GRAPH)
public double getVelocity() { return velocity; }

DashboardManager.autoPopulate(this);
```

**Benefits:** Always up-to-date dashboard, less manual configuration, better driver feedback

### 9. Performance Monitor (`diagnostics/`)
Track system health and bottlenecks:

```java
PerformanceMonitor.getInstance()
    .watchCPU()
    .watchMemory()
    .watchCANUtilization()
    .alertOn(threshold -> threshold.cpuUsage > 80);
```

**Benefits:** Prevent brownouts, identify performance issues, optimize loop times

### 10. Safety Guards (`safety/`)
Automatic protection against common failure modes:

```java
SafetyManager.register(
    BrownoutGuard.reduceNonCriticalLoad(),
    CurrentLimitGuard.forMotor(shooterMotor, 40.0),
    SoftLimitValidator.checkAll()
);
```

**Benefits:** Prevent hardware damage, catch configuration errors, safer operation

---

## 📚 Documentation

- **[FEATURES.md](FEATURES.md)** - Detailed feature documentation with examples
- **[MIGRATION.md](MIGRATION.md)** - Upgrading from previous versions
- **[EXAMPLES.md](EXAMPLES.md)** - Common use cases and patterns
- **[API.md](API.md)** - Complete API reference

---

## 🎯 Competitive Advantages

Excalib provides features that many top-tier FRC teams have invested thousands of hours developing:

✅ **Auto-tuning** - What teams like 254 and 1678 use for quick robot bringup  
✅ **State machines** - Clean autonomous code like 971 and 2910  
✅ **Telemetry** - Real-time diagnostics like 6328's AdvantageKit  
✅ **Simulation** - Unit testing infrastructure like 2910 and 254  
✅ **Vision pipelines** - Proven filtering like 971 and 254  
✅ **Performance monitoring** - System health tracking like 1678  
✅ **Motion profiling** - Smooth movement like top-tier teams  

---

## 🔧 Quick Start

1. **Add Excalib to your robot project** - Already included in this repository
2. **Import utilities** - `import frc.excalib.*;`
3. **Use pre-built mechanisms** - Extend `Mechanism` or use `Arm`, `LinearExtension`, etc.
4. **Add telemetry** - Annotate fields with `@Telemetry`
5. **Build state machines** - Use `StateMachine.builder()` for autonomous
6. **Enable safety guards** - Register guards in `Robot.robotInit()`

---

## 🤝 Contributing

Features are prioritized based on competitive advantage and developer productivity. See `CONTRIBUTING.md` for guidelines.

---

## 📄 License

See `WPILib-License.md` for licensing information.

---

## 🏆 Used By

Team 6738 Excalibur - 2025 Reefscape Season

---

**Built by FRC teams, for FRC teams. Ship faster, compete better.**
