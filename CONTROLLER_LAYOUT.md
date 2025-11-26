# PS5 Controller Layout for Example Subsystems with Swerve

```
                    ╔════════════════════════════════════════════╗
                    ║     PS5 Controller (Port 0) - UPDATED     ║
                    ║         WITH SWERVE DRIVE SUPPORT         ║
                    ╚════════════════════════════════════════════╝

                        [Create] [Options]
                          [PS] [Touchpad]
                                │     │
                  ┌─────────────┴─────┴─────────────┐
                  │                                   │
                  │    D-Pad        [△○□✕]   Bumpers │
                  │      🎮           🎮        🎮    │
                  │                                   │
                  │  [LStick]              [RStick]   │
                  │     🕹️                   🕹️       │
                  │                                   │
                  └───────────────────────────────────┘
                      [L2]                    [R2]

═══════════════════════════════════════════════════════════════

🎮 FACE BUTTONS (Right Side)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    △ (Triangle)  →  Reset gyro heading (tank drive)
    │
    □   ○         →  □ (Square): Stop tank drive motors
    │   │         →  ○ (Circle): Arm to 0° horizontal (hold)
    ✕ (Cross)     →  Arm to 45° (hold)

═══════════════════════════════════════════════════════════════

🎮 D-PAD (Left Side) - TURRET CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

         ↑
    Turret to 0°
         │
    ← ───┼─── →
    -90°  │   +90°
         │
         ↓
      Turret to 180°

═══════════════════════════════════════════════════════════════

🕹️ LEFT STICK - SWERVE DRIVE (PRIMARY)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    Forward/Backward  →  Y-axis (field-centric forward/back at 80% speed)
    Left/Right        →  X-axis (field-centric strafe at 80% speed)
    
    ⚠️  NOTE: This controls the SWERVE DRIVE subsystem (4-wheel holonomic)
           Field-centric means forward is always away from driver station

═══════════════════════════════════════════════════════════════

🕹️ RIGHT STICK - ROTATION CONTROL (SWERVE)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    X-axis (left/right)  →  Rotate robot (60% angular speed)
    
    ⚠️  NOTE: Rotation works for both swerve and tank drive
           Tank drive uses 30% speed, swerve uses 60%

═══════════════════════════════════════════════════════════════

🎮 BUMPERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [L1] Left Bumper   →  Arm to 90° vertical (hold)
    [R1] Right Bumper  →  Arm manual control (use R2 for voltage)

═══════════════════════════════════════════════════════════════

🎮 TRIGGERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [L2] Left Trigger   →  (Available for custom mapping)
    [R2] Right Trigger  →  Control arm voltage (when R1 is held)

═══════════════════════════════════════════════════════════════

🎮 CENTER BUTTONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [Create]   (◄◄)  →  Stop flywheel
    [Options]  (►)   →  Run flywheel at 50 RPS (hold)
    [PS]       (⊙)   →  Stop swerve drive
    [Touchpad] (▭)   →  Reset swerve pose to (0,0)

═══════════════════════════════════════════════════════════════

📊 REAL-TIME FEEDBACK (NetworkTables)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Open Shuffleboard or Glass to see:
  • Gyro heading (degrees)
  • Gyro connection status
  • Arm position (degrees) 
  • Arm at setpoint indicator
  • Turret angle (degrees)
  • Turret at setpoint indicator
  • Flywheel velocity (RPS)
  • Flywheel at speed indicator

═══════════════════════════════════════════════════════════════

💡 TIPS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

• Hold buttons to maintain position (arm, turret)
• Use gentle stick movements for smooth control
• Watch NetworkTables for real-time feedback
• Check console output for command status
• Enable the robot in Driver Station to start

═══════════════════════════════════════════════════════════════

📝 TESTING CHECKLIST
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Drive System:
  □ Left stick moves robot forward/backward
  □ Left stick strafes robot left/right  
  □ Right stick rotates robot
  □ A button resets gyro heading
  □ B button stops drive

Arm System:
  □ Y button moves arm to 45°
  □ X button moves arm to 0°
  □ LB moves arm to 90°
  □ RB + RT manual control works
  □ Arm holds position when released

Turret System:
  □ D-pad Up rotates to 0°
  □ D-pad Right rotates to 90°
  □ D-pad Down rotates to 180°
  □ D-pad Left rotates to -90°
  □ Turret motion is smooth (profiled)

Flywheel System:
  □ Start button runs flywheel
  □ Back button stops flywheel
  □ Velocity is logged correctly

Motor System:
  □ Left trigger runs motor forward

AdvantageKit Logging:
  □ All telemetry appears in NetworkTables
  □ Log files are created
  □ Can replay logs in AdvantageScope

═══════════════════════════════════════════════════════════════
```
