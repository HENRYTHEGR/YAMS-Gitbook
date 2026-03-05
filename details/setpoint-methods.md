---
icon: sliders
---

# Setpoint Methods vs Command Helpers

YAMS Mechanism classes provide two ways to control mechanisms: **direct setpoint methods** inherited from `SmartMechanism`, and **command helper methods** that return `Command` objects. Understanding when to use each is key to writing clean, effective robot code.

## Overview

| Approach | Example | Returns | Use Case |
|----------|---------|---------|----------|
| **Command Helpers** | `arm.run(Degrees.of(45))` | `Command` | Button bindings, autonomous sequences |
| **Direct Setpoint Methods** | `arm.setMechanismPositionSetpoint(Degrees.of(45))` | `void` | Custom commands, AdvantageKit IO, low-level control |

## Command Helper Methods

Command helpers are methods on Mechanism classes that return `Command` objects. These are the **recommended approach** for most use cases.

### Why Use Command Helpers?

1. **Subsystem Requirements** - Commands automatically require their subsystem, preventing conflicts
2. **Lifecycle Management** - Commands handle starting/stopping the closed loop controller
3. **Composability** - Easy to chain with `.andThen()`, `.alongWith()`, etc.
4. **Trigger Integration** - Work directly with button triggers like `controller.a().onTrue()`
5. **Naming** - Commands are automatically named for debugging

### Available Command Helpers by Mechanism

#### Arm / Pivot

```java
// Run continuously to an angle (use for teleop)
arm.run(Degrees.of(45))
arm.run(() -> joystick.getY() * 90)  // Supplier version
arm.setAngle(Degrees.of(45))         // Alias for run()

// Run to angle and end when reached (use in autonomous sequences)
arm.runTo(Degrees.of(45), Degrees.of(1))  // with tolerance

// Open loop control
arm.set(0.5)                         // Duty cycle [-1, 1]
arm.set(() -> joystick.getY())       // Duty cycle supplier
arm.setVoltage(Volts.of(6))          // Voltage
arm.setVoltage(() -> Volts.of(6))    // Voltage supplier
```

#### Elevator

```java
// Run continuously to a height (use for teleop)
elevator.run(Meters.of(1.0))
elevator.run(() -> Meters.of(joystick.getY()))
elevator.setHeight(Meters.of(1.0))   // Alias for run()

// Run to height and end when reached (use in autonomous sequences)
elevator.runTo(Meters.of(1.0), Centimeters.of(2))  // with tolerance

// Open loop control
elevator.set(0.5)
elevator.setVoltage(Volts.of(6))
```

#### FlyWheel

```java
// Run continuously at a velocity (use for teleop)
flywheel.run(RPM.of(5000))
flywheel.run(() -> RPM.of(targetRPM))
flywheel.setSpeed(RPM.of(5000))      // Alias for run()

// Run to velocity and end when reached (use for spin-up sequences)
flywheel.runTo(RPM.of(5000), RPM.of(100))  // with tolerance

// Linear velocity (requires circumference configured)
flywheel.run(MetersPerSecond.of(20))
flywheel.runTo(MetersPerSecond.of(20), MetersPerSecond.of(0.5))

// Open loop control
flywheel.set(0.8)
flywheel.setVoltage(Volts.of(10))
```

### Example: Button Bindings

```java
// Teleop controls - use run() for continuous control
controller.a().whileTrue(arm.run(Degrees.of(45)));
controller.b().whileTrue(arm.run(Degrees.of(-30)));
controller.x().whileTrue(elevator.run(Meters.of(1.0)));
controller.y().whileTrue(flywheel.run(RPM.of(5000)));

// Autonomous-style - use runTo() when you need to wait for completion
controller.leftBumper().onTrue(
    arm.runTo(Degrees.of(90), Degrees.of(2))
        .andThen(shooter.runTo(RPM.of(5000), RPM.of(50)))
        .andThen(intake.feedNote())
);
```

## Direct Setpoint Methods

Direct setpoint methods are inherited from `SmartMechanism` and set the motor controller's target immediately without returning a Command.

### SmartMechanism Setpoint Methods

All Mechanism classes inherit these methods from `SmartMechanism`:

```java
// Position setpoints
void setMechanismPositionSetpoint(Angle angle)      // Rotational position (Arm, Pivot)
void setMeasurementPositionSetpoint(Distance dist)  // Linear position (Elevator)

// Velocity setpoints
void setMechanismVelocitySetpoint(AngularVelocity vel)  // Rotational velocity
void setMeasurementVelocitySetpoint(LinearVelocity vel) // Linear velocity (Elevator, FlyWheel)

// Open loop setpoints
void setMechanismVoltageSetpoint(Voltage volts)     // Voltage control
void setDutyCycleSetpoint(double dutyCycle)         // Duty cycle [-1, 1]
```

### When to Use Direct Setpoint Methods

1. **AdvantageKit IO Classes** - When implementing IO interfaces that don't use Commands
2. **Custom Command Logic** - When building complex commands with custom execute() methods
3. **State Machine Control** - When a state machine directly controls mechanism state
4. **Performance Critical** - Avoiding Command overhead in tight loops (rare)

### Example: AdvantageKit IO

```java
public class ArmIOYAMS implements ArmIO {
  private final Arm arm;

  @Override
  public void setTargetAngle(double rotations) {
    // Use direct setpoint method - no Command needed
    arm.setMechanismPositionSetpoint(Rotations.of(rotations));
  }
}
```

### Example: Custom Command

```java
public class ComplexArmCommand extends Command {
  private final Arm arm;
  private final Supplier<Angle> targetSupplier;

  @Override
  public void execute() {
    Angle target = targetSupplier.get();
    // Apply custom logic before setting
    if (arm.getAngle().lt(Degrees.of(0))) {
      target = target.plus(Degrees.of(5)); // Offset when below horizontal
    }
    arm.setMechanismPositionSetpoint(target);
  }
}
```

## Setpoint Method Support Matrix

Not all setpoint methods are supported by all Mechanism classes. Methods that don't apply throw `UnsupportedOperationException` or `RuntimeException`.

### Position Setpoint Methods

| Method | Arm | Pivot | Elevator | FlyWheel |
|--------|-----|-------|----------|----------|
| `setMechanismPositionSetpoint(Angle)` | Yes | Yes | Yes | No |
| `setMeasurementPositionSetpoint(Distance)` | No | No | Yes | No |

### Velocity Setpoint Methods

| Method | Arm | Pivot | Elevator | FlyWheel |
|--------|-----|-------|----------|----------|
| `setMechanismVelocitySetpoint(AngularVelocity)` | Yes | Yes | Yes | Yes |
| `setMeasurementVelocitySetpoint(LinearVelocity)` | No | No | Yes | Yes* |

*FlyWheel requires circumference to be configured for linear velocity.

### Open Loop Setpoint Methods

| Method | Arm | Pivot | Elevator | FlyWheel |
|--------|-----|-------|----------|----------|
| `setMechanismVoltageSetpoint(Voltage)` | Yes | Yes | Yes | Yes |
| `setDutyCycleSetpoint(double)` | Yes | Yes | Yes | Yes |

{% hint style="warning" %}
**Unsupported Methods**: Calling an unsupported method (like `setMeasurementPositionSetpoint(Distance)` on an `Arm`) will throw a `RuntimeException`. Check the matrix above before using direct setpoint methods.
{% endhint %}

## Checking Position with isNear()

To check if a mechanism is near a target position, use the `isNear()` method which returns a `Trigger`:

```java
// Arm / Pivot - uses Angle.isNear() internally
Trigger nearTarget = arm.isNear(Degrees.of(45), Degrees.of(2));

// Elevator - uses Distance.isNear() internally  
Trigger nearHeight = elevator.isNear(Meters.of(1.0), Centimeters.of(2));

// FlyWheel - uses AngularVelocity.isNear() internally
Trigger atSpeed = flywheel.isNear(RPM.of(5000), RPM.of(100));
```

These methods use WPILib's built-in `isNear()` function on the `Angle`, `Distance`, and `AngularVelocity` measure classes for accurate tolerance checking.

### Using isNear() in Commands

```java
// Wait until arm is near target
Commands.waitUntil(arm.isNear(Degrees.of(45), Degrees.of(1)))

// Conditional logic based on position
arm.isNear(Degrees.of(90), Degrees.of(5)).onTrue(ledSubsystem.setGreen());

// In a command's isFinished()
@Override
public boolean isFinished() {
    return arm.isNear(targetAngle, tolerance).getAsBoolean();
}
```

## Decision Guide

```
                    ┌─────────────────────────┐
                    │ How are you controlling │
                    │    the mechanism?       │
                    └───────────┬─────────────┘
                                │
              ┌─────────────────┴─────────────────┐
              │                                   │
              ▼                                   ▼
    ┌─────────────────┐                ┌─────────────────┐
    │ Button binding  │                │ Custom command  │
    │ or autonomous   │                │ or IO class     │
    │ sequence?       │                │                 │
    └────────┬────────┘                └────────┬────────┘
             │                                  │
             ▼                                  ▼
    ┌─────────────────┐                ┌─────────────────┐
    │ Use Command     │                │ Use Direct      │
    │ Helpers         │                │ Setpoint        │
    │                 │                │ Methods         │
    │ arm.run()       │                │                 │
    │ arm.runTo()     │                │ arm.setMechanism│
    │ arm.setAngle()  │                │ PositionSetpoint│
    └─────────────────┘                └─────────────────┘
```

## Best Practices

1. **Prefer Command Helpers** - They handle subsystem requirements and closed loop lifecycle
2. **Use run() for Teleop** - Continuous control that responds to interruption
3. **Use runTo() for Sequences** - When you need to wait for the mechanism to reach a target
4. **Use Direct Methods for IO** - When implementing AdvantageKit IO or custom commands
5. **Always Check Support** - Verify the method is supported before using direct setpoint methods
6. **Use Triggers for State** - Use `isNear()`, `max()`, `min()` triggers instead of polling

## See Also

- [run() vs runTo() Commands](run-vs-runto.md) - Important differences between continuous and goal-based commands
- [AdvantageKit Integration](advantagekit-integration.md) - Using YAMS with AdvantageKit's IO pattern
- [Live Tuning](integrations.md) - Units for all tunable parameters
