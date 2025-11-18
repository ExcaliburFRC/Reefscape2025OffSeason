# Excalib Library - Usage Examples

Practical examples demonstrating how to use Excalib's productivity features.

---

## Command Decorators

### Example 1: Robust Intake with Retry and Timeout
```java
public class RobotContainer {
    private final IntakeSubsystem intake = new IntakeSubsystem();
    
    public Command getAutonomousCommand() {
        // Create a robust intake command that:
        // - Retries up to 3 times if it fails
        // - Times out after 5 seconds total
        Command robustIntake = new TimeoutCommand(
            new RetryCommand(intake.intakeGamePiece(), 3),
            5.0
        );
        
        return Commands.sequence(
            robustIntake,
            scoreGamePiece()
        );
    }
}
```

### Example 2: Rate-Limited Vision Processing
```java
public class VisionSubsystem extends SubsystemBase {
    public Command trackTarget() {
        // Vision processing is expensive, limit to 20 Hz
        return new RateLimitedCommand(
            new RunCommand(this::processVisionFrame, this),
            20.0
        );
    }
    
    private void processVisionFrame() {
        // Heavy vision processing here
    }
}
```

---

## Telemetry System

### Example 1: Subsystem Telemetry
```java
public class ShooterSubsystem extends SubsystemBase {
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    @Override
    public void periodic() {
        // Record values
        telemetry.recordValue("shooter/velocity_rpm", getVelocity());
        telemetry.recordValue("shooter/target_rpm", getTargetVelocity());
        telemetry.recordBoolean("shooter/at_speed", isAtSpeed());
        
        // Track if we're ready to shoot
        if (isAtSpeed()) {
            telemetry.recordEvent("shooter/state", "ready");
        }
    }
}
```

### Example 2: Measuring Operation Latency
```java
public class VisionSubsystem extends SubsystemBase {
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    public Optional<VisionTarget> processFrame() {
        // Start timing
        telemetry.startLatencyTimer("vision/processing");
        
        // Do expensive vision processing
        Optional<VisionTarget> target = detectTarget();
        
        // Record how long it took
        double latency = telemetry.recordLatency("vision/processing");
        
        if (latency > 0.05) { // 50ms
            System.out.println("WARNING: Vision processing slow: " + latency * 1000 + "ms");
        }
        
        return target;
    }
}
```

---

## Auto-Tuning

### Example 1: Tune a Shooter Flywheel
```java
public class ShooterSubsystem extends SubsystemBase {
    private final Motor shooterMotor;
    
    public Command autoTuneShooter() {
        return Commands.runOnce(() -> {
            System.out.println("Starting auto-tune...");
            
            AutoTuner tuner = new AutoTuner(
                shooterMotor,
                shooterMotor::getMotorVelocity
            );
            
            Gains gains = tuner.characterizeAndTune(
                AutoTuneMethod.ZIEGLER_NICHOLS,
                AutoTuneConfig.forVelocityControl()
            );
            
            System.out.println("Calculated gains:");
            System.out.println("  kP: " + gains.kp);
            System.out.println("  kI: " + gains.ki);
            System.out.println("  kD: " + gains.kd);
            System.out.println("  kV: " + gains.kv);
            
            // TODO: Apply gains to motor controller
        });
    }
}
```

### Example 2: Tune an Arm Mechanism
```java
public class ArmSubsystem extends SubsystemBase {
    private final Motor armMotor;
    
    public void tuneArm() {
        AutoTuner tuner = new AutoTuner(
            armMotor,
            armMotor::getMotorPosition  // Position feedback for position control
        );
        
        // Use NO_OVERSHOOT method for safety - arm shouldn't overshoot
        Gains gains = tuner.characterizeAndTune(
            AutoTuneMethod.NO_OVERSHOOT,
            AutoTuneConfig.forPositionControl()
        );
        
        // Apply gains...
    }
}
```

---

## Safety System

### Example 1: Robot-Wide Safety Setup
```java
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        
        // Register safety guards
        SafetyManager.register(
            // Brownout protection
            BrownoutGuard.builder()
                .withWarningVoltage(11.5)
                .withCriticalVoltage(10.5)
                .onWarning(() -> {
                    System.out.println("Low battery warning!");
                    // Could disable LEDs, reduce motor currents, etc.
                })
                .onCritical(() -> {
                    System.err.println("CRITICAL: Battery voltage too low!");
                    // Emergency shutdown of non-essential systems
                })
                .build(),
            
            // Motor current protection
            CurrentLimitGuard.forMotor(robotContainer.getIntakeMotor(), 40.0),
            CurrentLimitGuard.forMotor(robotContainer.getShooterMotor(), 60.0),
            CurrentLimitGuard.forMotor(robotContainer.getClimberMotor(), 80.0, 2.0) // 80A for 2 seconds
        );
    }
    
    @Override
    public void robotPeriodic() {
        // Check all safety guards every loop
        SafetyManager.getInstance().checkAll();
    }
}
```

### Example 2: Subsystem-Specific Guards
```java
public class ClimberSubsystem extends SubsystemBase {
    private final Motor climberMotor;
    
    public ClimberSubsystem(Motor motor) {
        this.climberMotor = motor;
        
        // Register a high current limit guard specifically for climbing
        SafetyManager.register(
            CurrentLimitGuard.forMotor(climberMotor, 80.0, 2.0)
        );
    }
}
```

---

## Performance Monitoring

### Example 1: Basic Performance Monitoring
```java
public class Robot extends TimedRobot {
    private final PerformanceMonitor perfMonitor = PerformanceMonitor.getInstance();
    
    @Override
    public void robotInit() {
        // Enable all monitoring
        perfMonitor.watchCPU()
                   .watchMemory()
                   .watchCANUtilization();
    }
    
    @Override
    public void robotPeriodic() {
        perfMonitor.startCycle();
        
        // Your periodic code here
        CommandScheduler.getInstance().run();
        
        perfMonitor.endCycle();
        perfMonitor.update();
    }
}
```

### Example 2: Performance Analysis
```java
public class PerformanceAnalyzer {
    public static void printStats() {
        PerformanceMonitor pm = PerformanceMonitor.getInstance();
        
        System.out.println("=== Performance Statistics ===");
        System.out.println("Average cycle time: " + 
            pm.getAverageCycleTime() * 1000 + " ms");
        System.out.println("Max cycle time: " + 
            pm.getMaxCycleTime() * 1000 + " ms");
        System.out.println("Min cycle time: " + 
            pm.getMinCycleTime() * 1000 + " ms");
        
        if (pm.getMaxCycleTime() > 0.020) {
            System.err.println("WARNING: Detected slow loops!");
        }
    }
}
```

---

## Vision Utilities

### Example 1: Target Detection and Selection
```java
public class VisionSubsystem extends SubsystemBase {
    public Optional<VisionTarget> getBestTarget() {
        // Get all detected targets from your vision system
        List<VisionTarget> allTargets = detectAllTargets();
        
        // Apply filters to eliminate noise
        List<VisionTarget> filtered = TargetFilter.applyFilters(
            allTargets,
            TargetFilter.minArea(0.05),           // At least 5% of frame
            TargetFilter.minConfidence(0.75),      // At least 75% confidence
            TargetFilter.inXRange(-0.3, 0.3)       // Within center 60% horizontally
        );
        
        // Select the best target
        VisionTarget best = TargetFilter.selectClosestToCenter(filtered);
        
        return Optional.ofNullable(best);
    }
    
    private List<VisionTarget> detectAllTargets() {
        // Your vision detection code here
        return new ArrayList<>();
    }
}
```

### Example 2: Vision-Guided Alignment Command
```java
public class DriveToTargetCommand extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    
    public DriveToTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        Optional<VisionTarget> target = vision.getBestTarget();
        
        if (target.isPresent()) {
            VisionTarget t = target.get();
            
            // Calculate rotation to center target
            double rotationSpeed = t.getX() * 2.0; // Simple P controller
            
            drive.arcadeDrive(0, rotationSpeed);
        } else {
            drive.stop();
        }
    }
    
    @Override
    public boolean isFinished() {
        return vision.getBestTarget()
            .map(t -> Math.abs(t.getX()) < 0.05) // Aligned within 5%
            .orElse(false);
    }
}
```

---

## Combined Example: Complete Autonomous Routine

```java
public class CompleteAutoExample {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    public Command getAutonomousCommand() {
        return Commands.sequence(
            // Log autonomous start
            Commands.runOnce(() -> 
                telemetry.recordEvent("auto", "started")),
            
            // Drive to game piece with timeout
            new TimeoutCommand(
                drive.driveToPosition(/* ... */),
                3.0
            ),
            
            // Intake with retry (may fail if game piece isn't there)
            new TimeoutCommand(
                new RetryCommand(
                    intake.intakeGamePiece(),
                    2  // Try twice
                ),
                5.0
            ),
            
            // Drive to scoring position
            new TimeoutCommand(
                drive.driveToPosition(/* ... */),
                3.0
            ),
            
            // Align using vision (rate limited to reduce CPU usage)
            new TimeoutCommand(
                new RateLimitedCommand(
                    new AlignToTargetCommand(drive, vision),
                    20.0  // 20 Hz max
                ),
                2.0
            ),
            
            // Shoot
            shooter.shootCommand(),
            
            // Log completion
            Commands.runOnce(() -> 
                telemetry.recordEvent("auto", "completed"))
        );
    }
    
    private class AlignToTargetCommand extends Command {
        private final DriveSubsystem drive;
        private final VisionSubsystem vision;
        
        public AlignToTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
            this.drive = drive;
            this.vision = vision;
            addRequirements(drive);
        }
        
        @Override
        public void execute() {
            telemetry.startLatencyTimer("auto/vision_align");
            
            Optional<VisionTarget> target = vision.getBestTarget();
            if (target.isPresent()) {
                double rotation = target.get().getX() * 2.0;
                drive.arcadeDrive(0, rotation);
            }
            
            telemetry.recordLatency("auto/vision_align");
        }
        
        @Override
        public boolean isFinished() {
            return vision.getBestTarget()
                .map(t -> Math.abs(t.getX()) < 0.05)
                .orElse(false);
        }
    }
}
```

---

## Integration Tips

### Tip 1: Gradual Adoption
You don't have to use all features at once. Start with one or two:

1. **Week 1**: Add command decorators to your autonomous
2. **Week 2**: Add telemetry to key subsystems
3. **Week 3**: Add safety guards for motor protection
4. **Week 4**: Add performance monitoring

### Tip 2: Testing
Always test new features on a practice robot first:

```java
// Easy enable/disable for testing
private static final boolean ENABLE_NEW_FEATURES = true;

if (ENABLE_NEW_FEATURES) {
    SafetyManager.register(/* guards */);
}
```

### Tip 3: Competition Mode
You can disable features for competition if needed:

```java
@Override
public void robotInit() {
    // Disable telemetry in competition to save cycles
    if (DriverStation.isFMSAttached()) {
        TelemetryManager.getInstance().setEnabled(false);
    }
}
```

---

## Common Patterns

### Pattern 1: Telemetry + Safety
```java
public class MotorSubsystem extends SubsystemBase {
    private final Motor motor;
    private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
    public MotorSubsystem(Motor motor) {
        this.motor = motor;
        
        // Add safety guard
        SafetyManager.register(
            CurrentLimitGuard.forMotor(motor, 40.0)
        );
    }
    
    @Override
    public void periodic() {
        // Record telemetry
        telemetry.recordValue("motor/current", motor.getCurrent());
        telemetry.recordValue("motor/temperature", motor.getTemperature());
        
        // Check if safety guards triggered
        if (motor.getCurrent() > 35.0) {
            telemetry.recordEvent("motor", "high_current_warning");
        }
    }
}
```

### Pattern 2: Auto-Tune During Practice
```java
public class RobotContainer {
    // Button binding for auto-tune
    private void configureButtonBindings() {
        // Hold button 10 for 3 seconds to auto-tune shooter
        new Trigger(() -> 
            operator.getRawButton(10) && 
            DriverStation.isDisabled()
        ).whileTrue(shooter.autoTuneCommand());
    }
}
```

---

**For more examples, see the `examples/` directory in the repository.**
