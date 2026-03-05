---
icon: play
---

# run() vs runTo() Commands

Understanding when to use `run()` versus `runTo()` is critical for building reliable robot control. Choosing the wrong one leads to unexpected behavior, especially when combined with default commands.

## Quick Reference

| Method | Command Ends? | Motor Stops? | Best For |
|--------|--------------|--------------|----------|
| `run(setpoint)` | Never (runs until interrupted) | No | Continuous control, teleop |
| `runTo(setpoint)` | When goal reached | **No** | Autonomous sequences, one-shot movements |

{% hint style="danger" %}
**Critical**: Neither `run()` nor `runTo()` stops the motor when the command ends. The `SmartMotorController` continues sending the last request. This is by design for smooth control, but can cause unexpected behavior if not understood.
{% endhint %}

## run() - Continuous Control

Use `run()` when you expect the command to run **indefinitely** until interrupted by another command.

```java
/**
 * Set the height of the elevator. Command runs forever until interrupted.
 * @param height Target height
 * @return A Command that never ends on its own
 */
public Command setHeight(Distance height) { 
  return elevator.run(height);
}
```

### When to Use run()

- **Teleop button bindings** with `whileTrue()` - command runs while button is held
- **Default commands** - these should run forever by definition
- **Holding position** - when you want to actively maintain a setpoint
- **Velocity control** - flywheels that spin until told otherwise

### Example: Teleop Elevator Control

```java
// RobotContainer.java
public RobotContainer() {
  // Default command: always try to go to home position
  m_elevator.setDefaultCommand(m_elevator.setHeight(Meters.of(0)));
}

private void configureBindings() {
  // While A is held, go to 0.5m. When released, default command takes over.
  m_driverController.a().whileTrue(m_elevator.setHeight(Meters.of(0.5)));
  
  // While B is held, go to 1.0m. When released, default command takes over.
  m_driverController.b().whileTrue(m_elevator.setHeight(Meters.of(1.0)));
}
```

**What happens:**
1. Robot starts, default command runs, elevator goes to 0m
2. Driver presses A, `setHeight(0.5m)` interrupts default, elevator goes to 0.5m
3. Driver releases A, command is cancelled, default command resumes, elevator returns to 0m

This works correctly because `run()` never ends on its own - it's always interrupted by something else.

## runTo() - Goal-Based Control

Use `runTo()` when you want the command to **end automatically** when the mechanism reaches its goal.

```java
/**
 * Set the height of the elevator. Command ends when goal is reached.
 * @param height Target height
 * @param tolerance Distance tolerance for completion
 * @return A Command that ends when at setpoint
 */
public Command setHeightAndStop(Distance height, Distance tolerance) { 
  return elevator.runTo(height, tolerance);
}
```

### When to Use runTo()

- **Autonomous routines** - move to position, then do the next thing
- **Sequential commands** - `moveArm.andThen(shoot).andThen(moveArmBack)`
- **One-shot movements** - press button once to go to a preset

{% hint style="warning" %}
**Important**: `runTo()` requires a tolerance parameter to determine when the goal is reached. Choose a tolerance appropriate for your mechanism (e.g., `Centimeters.of(2)` for an elevator, `Degrees.of(1)` for an arm).
{% endhint %}

### Example: Autonomous Sequence

```java
// Autonomous command sequence
public Command autoScoreHigh() {
  return Commands.sequence(
    // Move elevator to scoring position (ends when reached)
    m_elevator.setHeightAndStop(Meters.of(1.2), Centimeters.of(2)),
    // Extend arm (ends when reached)
    m_arm.setAngleAndStop(Degrees.of(45), Degrees.of(1)),
    // Score the game piece
    m_intake.outtake().withTimeout(0.5),
    // Retract arm (ends when reached)
    m_arm.setAngleAndStop(Degrees.of(0), Degrees.of(1)),
    // Lower elevator (ends when reached)
    m_elevator.setHeightAndStop(Meters.of(0), Centimeters.of(2))
  );
}
```

## The Default Command Pitfall

{% hint style="danger" %}
**This is the most common mistake with `runTo()`!**
{% endhint %}

When `runTo()` finishes (because the goal was reached), the command scheduler looks for another command to run. If you have a **default command** set, it will immediately take over.

### The Problem

```java
// RobotContainer.java - PROBLEMATIC CODE
public RobotContainer() {
  // Default command: go to home position
  m_elevator.setDefaultCommand(m_elevator.setHeight(Meters.of(0)));
}

private void configureBindings() {
  // Press A once to go to 1 meter
  m_driverController.a().onTrue(m_elevator.setHeightAndStop(Meters.of(1.0)));
}
```

**What the programmer expects:**
1. Press A
2. Elevator goes to 1 meter
3. Elevator stays at 1 meter

**What actually happens:**
1. Press A
2. `setHeightAndStop(1.0m)` starts running
3. Elevator moves toward 1 meter
4. Elevator reaches 1 meter, `runTo()` command ends
5. **Default command immediately starts**, commanding 0 meters
6. Elevator goes back down to 0 meters

### Solution 1: Use run() Instead

If you want the mechanism to stay at the commanded position, use `run()`:

```java
// Press A once to go to 1 meter and STAY there
m_driverController.a().onTrue(m_elevator.setHeight(Meters.of(1.0)));
```

Now the `run()` command never ends on its own, so the default command never gets a chance to run. The elevator will stay at 1 meter until another command is scheduled.

### Solution 2: No Default Command

If you don't need a default command, don't set one:

```java
public RobotContainer() {
  // No default command - mechanism holds last setpoint
  // m_elevator.setDefaultCommand(...); // Removed!
}

private void configureBindings() {
  // These work correctly now
  m_driverController.a().onTrue(m_elevator.setHeightAndStop(Meters.of(0.5)));
  m_driverController.b().onTrue(m_elevator.setHeightAndStop(Meters.of(1.0)));
}
```

Since there's no default command, when `runTo()` ends, nothing else runs. The `SmartMotorController` keeps sending the last setpoint, holding position.

### Solution 3: Use whileTrue() for Momentary Control

If you want the mechanism to return to a home position when not commanded, use `whileTrue()`:

```java
public RobotContainer() {
  m_elevator.setDefaultCommand(m_elevator.setHeight(Meters.of(0)));
}

private void configureBindings() {
  // HOLD A to go to 1 meter, RELEASE to return home
  m_driverController.a().whileTrue(m_elevator.setHeight(Meters.of(1.0)));
}
```

This makes the intent clear: hold the button to hold the position.

### Solution 4: Use Triggers for State-Based Control

Following [command-based best practices](https://bovlb.github.io/frc-tips/commands/best-practices.html), use triggers to control behavior:

```java
public class ElevatorSubsystem extends SubsystemBase {
  // Expose state as triggers
  public final Trigger isHigh = new Trigger(() -> elevator.getPosition().gt(Meters.of(0.8)));
  public final Trigger isLow = new Trigger(() -> elevator.getPosition().lt(Meters.of(0.2)));
  
  // Command factories
  public Command goHigh() { return elevator.run(Meters.of(1.0)); }
  public Command goLow() { return elevator.run(Meters.of(0)); }
}

// RobotContainer.java
private void configureBindings() {
  // Button toggles between high and low
  m_driverController.a()
    .and(m_elevator.isLow)
    .onTrue(m_elevator.goHigh());
    
  m_driverController.a()
    .and(m_elevator.isHigh)
    .onTrue(m_elevator.goLow());
}
```

## Understanding Motor Behavior

Both `run()` and `runTo()` set the `SmartMotorController`'s setpoint. When the command ends:

- The **command** stops running
- The **SmartMotorController** does NOT stop
- The motor continues driving toward the last setpoint

This is intentional! It allows for smooth transitions between commands without the mechanism "going limp" between commands.

### If You Need to Stop the Motor

If you actually want to stop motor output when a command ends, use `finallyDo()`:

```java
public Command setHeightAndStopMotor(Distance height, Distance tolerance) {
  return elevator.runTo(height, tolerance)
    .finallyDo(() -> motor.stopMotor()); // Actually stops the motor
}
```

Or create a dedicated stop command:

```java
public Command stop() {
  return runOnce(() -> {
    motor.set(0); // Open loop 0
    // Or: motor.stopMotor();
  });
}
```

## Autonomous with runTo()

`runTo()` shines in autonomous routines where you need sequential operations:

```java
public Command autoRoutine() {
  return Commands.sequence(
    // Step 1: Position the arm (runTo ends when reached)
    m_arm.setAngleAndStop(Degrees.of(45), Degrees.of(1)),
    
    // Step 2: Start shooter (run() because we want it to keep running)
    m_shooter.setSpeed(RPM.of(4000)),
    
    // Step 3: Wait for shooter to spin up
    Commands.waitUntil(() -> m_shooter.isNear(RPM.of(4000), RPM.of(100))),
    
    // Step 4: Feed the game piece
    m_feeder.feedForward().withTimeout(0.5),
    
    // Step 5: Stop shooter and return arm
    Commands.parallel(
      m_shooter.setVoltage(Volts.of(0)),  // Stop the shooter
      m_arm.setAngleAndStop(Degrees.of(0), Degrees.of(1))
    )
  );
}
```

## Common Pitfalls Summary

| Pitfall | Symptom | Solution |
|---------|---------|----------|
| `runTo()` with default command | Mechanism returns to default position after reaching goal | Use `run()` or remove default command |
| Using `onTrue()` with `run()` expecting it to end | Command runs forever, can't do anything else | Use `runTo()` for finite commands |
| Assuming motor stops when command ends | Mechanism drifts or holds unexpected position | Understand that `SmartMotorController` continues |
| No tolerance set | `runTo()` never ends because goal is never "reached" | Set `withClosedLoopTolerance()` in config |
| Tolerance too tight | `runTo()` ends and restarts repeatedly (oscillation) | Increase tolerance or add debounce |

## Decision Flowchart

```
Do you want the command to end automatically?
├── NO → Use run()
│   └── Combine with whileTrue() for teleop
│   └── Use as default command
└── YES → Use runTo()
    └── Is there a default command?
        ├── YES → Will it conflict with your goal?
        │   ├── YES → Remove default or use run() instead
        │   └── NO → runTo() is fine
        └── NO → runTo() is fine
```

## Best Practices

1. **Default commands should use `run()`** - They run forever by definition
2. **Teleop with `whileTrue()` should use `run()`** - Natural pairing
3. **Autonomous sequences should use `runTo()`** - Need to know when steps complete
4. **Be explicit about intent** - Name methods clearly (`setHeight` vs `setHeightAndStop`)
5. **Consider state-based control** - Use triggers instead of complex command logic
6. **Test both paths** - Test what happens when commands end, not just when they run

## Further Reading

- [Command-Based Best Practices](https://bovlb.github.io/frc-tips/commands/best-practices.html) - Comprehensive guide on command-based architecture
- [WPILib Command Scheduler](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html) - How commands are scheduled
- [Default Commands](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html#default-commands) - Official documentation on default commands
