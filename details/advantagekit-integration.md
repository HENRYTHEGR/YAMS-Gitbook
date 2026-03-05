---
icon: database
---

# AdvantageKit Integration

YAMS integrates seamlessly with [AdvantageKit](https://docs.advantagekit.org/), Team 6328's logging and replay framework. This guide shows how to use YAMS mechanisms with AdvantageKit's IO layer pattern for deterministic log replay.

## What is AdvantageKit?

[AdvantageKit](https://docs.advantagekit.org/getting-started/what-is-advantagekit) is a logging framework that records **all inputs** to your robot code, enabling deterministic replay in simulation. Unlike traditional logging that captures specific values, AdvantageKit captures everything flowing into your code, allowing you to:

- Replay matches exactly as they happened
- Add new logged fields after the fact
- Test code changes against real match data
- Debug issues that only occurred on the real robot

{% hint style="info" %}
AdvantageKit is optional. YAMS works perfectly without it using its built-in telemetry system. Consider AdvantageKit if you want deterministic log replay capabilities.
{% endhint %}

## Architecture Overview

AdvantageKit uses an [IO interface pattern](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) to separate hardware interaction from business logic:

```
┌─────────────────────────────────────────────────────────────┐
│                      Subsystem                              │
│  (Business logic, commands, state management)               │
│                          │                                  │
│                          ▼                                  │
│                    IO Interface                             │
│              (Abstract input/output contract)               │
│                    /           \                            │
│                   /             \                           │
│              IO Real           IO Sim                       │
│         (Real hardware)    (Simulation)                     │
│               │                  │                          │
│               ▼                  ▼                          │
│     SmartMotorController   SmartMotorController             │
│        (TalonFXWrapper)      (SimWrapper)                   │
└─────────────────────────────────────────────────────────────┘
```

With YAMS, the `SmartMotorController` handles most of the complexity, so your IO implementations remain simple.

## Step 1: Define the IO Interface

Create an interface that defines what inputs your subsystem reads and what outputs it sends. Following AdvantageKit conventions, inputs are grouped in an `Inputs` class.

### Arm IO Interface

```java
package frc.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  /**
   * Inputs that will be logged and replayed.
   * The @AutoLog annotation generates ArmIOInputsAutoLogged class.
   */
  @AutoLog
  public static class ArmIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionRotations = 0.0;
    public boolean atGoal = false;
  }

  /** Update the inputs from hardware. Called every loop cycle. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Set the target angle for the arm. */
  default void setTargetAngle(double rotations) {}

  /** Stop the arm motor. */
  default void stop() {}
}
```

### Elevator IO Interface

```java
package frc.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionMeters = 0.0;
    public boolean atGoal = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetHeight(double meters) {}

  default void stop() {}
}
```

### Shooter IO Interface

```java
package frc.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetVelocityRotationsPerSec = 0.0;
    public boolean atGoal = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(double rotationsPerSec) {}

  default void stop() {}
}
```

{% hint style="info" %}
The `@AutoLog` annotation from AdvantageKit generates a class (e.g., `ArmIOInputsAutoLogged`) that handles serialization for log replay. See [AdvantageKit AutoLog documentation](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces#autolog) for details.
{% endhint %}

## Step 2: Implement the Real IO Class

The real IO implementation wraps a YAMS `SmartMotorController` and reads values from it.

### Arm IO Real Implementation

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SmartMotorControllerConfig;
import yams.mechanisms.config.SmartMotorControllerConfig.ControlMode;
import yams.mechanisms.config.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ArmIOReal implements ArmIO {

  private final SmartMotorController motor;

  public ArmIOReal(SubsystemBase subsystem, int canId) {
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withMomentOfInertia(Inches.of(18), Pounds.of(5))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(1.0, 2.0) // rot/s, rot/s²
        .withClosedLoopTolerance(Rotations.of(0.01))
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withTelemetry("Arm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    TalonFX talonFX = new TalonFX(canId);
    this.motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Read all values from SmartMotorController
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getAppliedVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().in(Amps);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations = motor.getClosedLoopTarget().in(Rotations);
    inputs.atGoal = motor.isNear(Rotations.of(0.01));
  }

  @Override
  public void setTargetAngle(double rotations) {
    motor.setTarget(Rotations.of(rotations));
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
```

### Elevator IO Real Implementation

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SmartMotorControllerConfig;
import yams.mechanisms.config.SmartMotorControllerConfig.ControlMode;
import yams.mechanisms.config.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorIOReal implements ElevatorIO {

  private final SmartMotorController motor;

  public ElevatorIOReal(SubsystemBase subsystem, int canId) {
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
        .withCircumference(Inches.of(1.5 * Math.PI)) // Pulley circumference
        .withCarriageMass(Pounds.of(10))
        .withClosedLoopController(10, 0, 0.5)
        .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
        .withTrapezoidalProfile(1.0, 2.0) // m/s, m/s²
        .withClosedLoopTolerance(Meters.of(0.01))
        .withSoftLimits(Meters.of(0), Meters.of(1.2))
        .withTelemetry("Elevator", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    TalonFX talonFX = new TalonFX(canId);
    this.motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionMeters = motor.getLinearPosition().in(Meters);
    inputs.velocityMetersPerSec = motor.getLinearVelocity().in(MetersPerSecond);
    inputs.appliedVolts = motor.getAppliedVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().in(Amps);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionMeters = motor.getClosedLoopTarget().in(Meters);
    inputs.atGoal = motor.isNear(Meters.of(0.01));
  }

  @Override
  public void setTargetHeight(double meters) {
    motor.setTarget(Meters.of(meters));
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
```

### Shooter IO Real Implementation

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SmartMotorControllerConfig;
import yams.mechanisms.config.SmartMotorControllerConfig.ControlMode;
import yams.mechanisms.config.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterIOReal implements ShooterIO {

  private final SmartMotorController motor;

  public ShooterIOReal(SubsystemBase subsystem, int canId) {
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive
        .withMomentOfInertia(Inches.of(4), Pounds.of(0.5))
        .withClosedLoopController(0.5, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
        .withClosedLoopTolerance(RotationsPerSecond.of(5))
        .withTelemetry("Shooter", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    TalonFX talonFX = new TalonFX(canId);
    this.motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getAppliedVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().in(Amps);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetVelocityRotationsPerSec = motor.getClosedLoopTarget().in(RotationsPerSecond);
    inputs.atGoal = motor.isNear(RotationsPerSecond.of(5));
  }

  @Override
  public void setTargetVelocity(double rotationsPerSec) {
    motor.setTarget(RotationsPerSecond.of(rotationsPerSec));
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
```

## Step 3: Implement the Sim IO Class

For simulation and log replay, create an IO implementation that doesn't touch real hardware. YAMS's `SimWrapper` handles physics simulation automatically.

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SmartMotorControllerConfig;
import yams.mechanisms.config.SmartMotorControllerConfig.ControlMode;
import yams.mechanisms.config.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SimWrapper;

public class ArmIOSim implements ArmIO {

  private final SmartMotorController motor;

  public ArmIOSim(SubsystemBase subsystem) {
    // Same config as real, but using SimWrapper
    SmartMotorControllerConfig config = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withMomentOfInertia(Inches.of(18), Pounds.of(5))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(1.0, 2.0)
        .withClosedLoopTolerance(Rotations.of(0.01))
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withTelemetry("Arm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    // SimWrapper for physics simulation
    this.motor = new SimWrapper(DCMotor.getKrakenX60(1), config);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getAppliedVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().in(Amps);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations = motor.getClosedLoopTarget().in(Rotations);
    inputs.atGoal = motor.isNear(Rotations.of(0.01));
  }

  @Override
  public void setTargetAngle(double rotations) {
    motor.setTarget(Rotations.of(rotations));
  }

  @Override
  public void stop() {
    motor.stop();
  }
}
```

{% hint style="success" %}
**Tip**: Extract the `SmartMotorControllerConfig` creation into a shared constants class so both Real and Sim implementations use identical configuration.
{% endhint %}

## Step 4: Create the Subsystem

The subsystem uses the IO interface and doesn't know whether it's running on real hardware or in simulation.

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  /**
   * Command to move the arm to a target angle.
   * Uses run() for continuous control.
   */
  public Command setAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations))
        .withName("Arm.setAngle(" + rotations + ")");
  }

  /**
   * Command to move the arm to a target angle and finish when reached.
   * Uses runTo() pattern - be careful with default commands!
   */
  public Command goToAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations))
        .until(() -> inputs.atGoal)
        .withName("Arm.goToAngle(" + rotations + ")");
  }

  /** Returns true if the arm is at its target position. */
  public boolean atGoal() {
    return inputs.atGoal;
  }

  /** Returns the current arm position in rotations. */
  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  /** Command to stop the arm. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Arm.stop");
  }
}
```

{% hint style="warning" %}
See [run() vs runTo() Commands](run-vs-runto.md) for important information about when commands end and how that interacts with default commands.
{% endhint %}

## Step 5: Wire It Up in RobotContainer

Use `Robot.isReal()` or AdvantageKit's mode detection to choose the appropriate IO implementation:

```java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.akit.*;

public class RobotContainer {

  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;
  private final ShooterSubsystem shooter;

  public RobotContainer() {
    // Create subsystems with appropriate IO implementation
    if (RobotBase.isReal()) {
      arm = new ArmSubsystem(new ArmIOReal(arm, 1));
      elevator = new ElevatorSubsystem(new ElevatorIOReal(elevator, 2));
      shooter = new ShooterSubsystem(new ShooterIOReal(shooter, 3));
    } else {
      arm = new ArmSubsystem(new ArmIOSim(arm));
      elevator = new ElevatorSubsystem(new ElevatorIOSim(elevator));
      shooter = new ShooterSubsystem(new ShooterIOSim(shooter));
    }

    configureBindings();
  }

  private void configureBindings() {
    // Commands work identically regardless of IO implementation
    controller.a().onTrue(arm.setAngle(0.25));
    controller.b().onTrue(arm.setAngle(-0.25));
    controller.x().onTrue(elevator.setHeight(1.0));
    controller.y().onTrue(shooter.spinUp(80)); // 80 rot/s
  }
}
```

## Log Replay Mode

For [log replay](https://docs.advantagekit.org/getting-started/what-is-advantagekit), AdvantageKit provides a "replay" mode where inputs are read from a log file instead of hardware. Create an empty IO implementation for this:

```java
package frc.robot.subsystems.akit;

/**
 * Empty IO implementation for log replay.
 * AdvantageKit replays the logged inputs, so no hardware interaction needed.
 */
public class ArmIOReplay implements ArmIO {
  // All methods use default implementations (do nothing)
  // AdvantageKit will inject the logged values into inputs
}
```

Then in RobotContainer, detect replay mode:

```java
switch (Constants.currentMode) {
  case REAL:
    arm = new ArmSubsystem(new ArmIOReal(arm, 1));
    break;
  case SIM:
    arm = new ArmSubsystem(new ArmIOSim(arm));
    break;
  case REPLAY:
    arm = new ArmSubsystem(new ArmIOReplay());
    break;
}
```

## Shared Configuration Pattern

To avoid duplicating configuration between Real and Sim implementations, extract it to a constants class:

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SmartMotorControllerConfig;
import yams.mechanisms.config.SmartMotorControllerConfig.ControlMode;
import yams.mechanisms.config.SmartMotorControllerConfig.MotorMode;

public class ArmConstants {

  public static SmartMotorControllerConfig createConfig(SubsystemBase subsystem) {
    return new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withMomentOfInertia(Inches.of(18), Pounds.of(5))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(1.0, 2.0)
        .withClosedLoopTolerance(Rotations.of(0.01))
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withTelemetry("Arm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  }
}
```

Then in your IO implementations:

```java
// In ArmIOReal
SmartMotorControllerConfig config = ArmConstants.createConfig(subsystem);
this.motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), config);

// In ArmIOSim
SmartMotorControllerConfig config = ArmConstants.createConfig(subsystem);
this.motor = new SimWrapper(DCMotor.getKrakenX60(1), config);
```

## Benefits of YAMS + AdvantageKit

| Feature | Benefit |
|---------|---------|
| Deterministic Replay | Debug issues from real matches in simulation |
| Unified Configuration | Same YAMS config works for real and sim |
| Physics Simulation | YAMS `SimWrapper` provides accurate mechanism physics |
| Automatic Logging | AdvantageKit logs all inputs automatically |
| Flexible IO | Swap hardware vendors without changing subsystem logic |

## Further Reading

- [AdvantageKit Documentation](https://docs.advantagekit.org/)
- [What is AdvantageKit?](https://docs.advantagekit.org/getting-started/what-is-advantagekit)
- [IO Interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces)
- [Template Projects](https://docs.advantagekit.org/getting-started/template-projects)
- [YAMS Example Code](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple_robot/src/main/java/frc/robot/subsystems/akit)
