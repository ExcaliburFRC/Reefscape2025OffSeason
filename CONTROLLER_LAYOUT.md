# Xbox Controller Layout for Example Subsystems

```
                    ╔════════════════════════════════════════════╗
                    ║         Xbox Controller (Port 0)          ║
                    ╚════════════════════════════════════════════╝

                              [Back] [Start]
                                │     │
                  ┌─────────────┴─────┴─────────────┐
                  │                                   │
                  │    D-Pad        [XYAB]   Bumpers │
                  │      🎮           🎮        🎮    │
                  │                                   │
                  │  [LStick]              [RStick]   │
                  │     🕹️                   🕹️       │
                  │                                   │
                  └───────────────────────────────────┘
                      [LT]                    [RT]

═══════════════════════════════════════════════════════════════

🎮 FACE BUTTONS (Right Side)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    Y Button (▲)  →  Arm to 45° (hold)
    │
    X   B         →  X: Arm to 0° (hold)
    │   │         →  B: Stop drive motors
    A             →  A: Reset gyro heading

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

🕹️ LEFT STICK - DRIVE CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    Forward/Backward  →  Y-axis (forward/backward at 50% speed)
    Left/Right        →  X-axis (strafe left/right at 50% speed)

═══════════════════════════════════════════════════════════════

🕹️ RIGHT STICK - ROTATION CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    X-axis (left/right)  →  Rotate robot (30% speed)

═══════════════════════════════════════════════════════════════

🎮 BUMPERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [LB] Left Bumper   →  Arm to 90° vertical (hold)
    [RB] Right Bumper  →  Arm manual control (use RT for voltage)

═══════════════════════════════════════════════════════════════

🎮 TRIGGERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [LT] Left Trigger   →  Run motor forward at 6V
    [RT] Right Trigger  →  Control arm voltage (when RB is held)

═══════════════════════════════════════════════════════════════

🎮 CENTER BUTTONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    [Back]  (◄◄)  →  Stop flywheel
    [Start] (►)   →  Run flywheel at 50 RPS (hold)

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
