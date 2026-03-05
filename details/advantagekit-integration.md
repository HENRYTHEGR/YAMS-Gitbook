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

AdvantageKit uses an [IO interface pattern](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) to separate hardware interaction from business logic. Traditionally, this requires separate IO implementations for real hardware and simulation:

```
┌─────────────────────────────────────────────────────────────┐
│  Traditional AdvantageKit Pattern (without YAMS)            │
│                                                             │
│                      Subsystem                              │
│                          │                                  │
│                    IO Interface                             │
│                    /           \                            │
│              IO Real           IO Sim                       │
│         (Real hardware)    (Physics sim)                    │
│          - Manual motor     - DCMotorSim                    │
│          - Manual encoders  - Manual state                  │
└─────────────────────────────────────────────────────────────┘
```

## YAMS Simplification: One IO Class for Real and Sim

**With YAMS, you don't need separate Real and Sim IO implementations.** Every `SmartMotorController` wrapper (TalonFXWrapper, SparkWrapper, etc.) and every Mechanism class (Arm, Elevator, FlyWheel, SwerveModule, SwerveDrive, etc.) includes **passive simulation** that runs automatically when `Robot.isSimulation()` returns true.

```
┌─────────────────────────────────────────────────────────────┐
│  YAMS Pattern: Single IO Implementation                     │
│                                                             │
│                      Subsystem                              │
│                          │                                  │
│                    IO Interface                             │
│                          │                                  │
│                    IO (Single!)  ◄── Same class for both!   │
│                          │                                  │
│               SmartMotorController                          │
│                    or Mechanism                             │
│                          │                                  │
│          ┌───────────────┴───────────────┐                  │
│          │                               │                  │
│    Real Hardware              Passive Simulation            │
│    (when deployed)            (when in sim)                 │
│                                                             │
│    Automatically selected based on Robot.isSimulation()    │
└─────────────────────────────────────────────────────────────┘
```

This means:
- **One IO class** handles both real robot and simulation
- **No duplicate code** - configuration is written once
- **No SimWrapper needed** - real hardware wrappers simulate themselves
- **Physics are automatic** - based on your DCMotor, gearing, and mass/MOI configuration

{% hint style="success" %}
**Key Insight**: When you create a `TalonFXWrapper`, `SparkWrapper`, `NovaWrapper`, or any YAMS Mechanism, the simulation physics run automatically in sim mode. You write your IO class once using real hardware objects, and YAMS handles the rest.
{% endhint %}

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
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(double rotationsPerSec) {}

  default void stop() {}
}
```

{% hint style="info" %}
The `@AutoLog` annotation from AdvantageKit generates a class (e.g., `ArmIOInputsAutoLogged`) that handles serialization for log replay. See [AdvantageKit AutoLog documentation](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces#autolog) for details.
{% endhint %}

## Step 2: Implement the IO Class

With YAMS, you only need **one IO implementation** that works for both real hardware and simulation. The `SmartMotorController` automatically detects when it's running in simulation and uses physics simulation instead of real hardware.

### Arm IO Implementation (Works for Real and Sim!)

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Single IO implementation for the arm - works in BOTH real and simulation!
 * Uses YAMS Arm mechanism class while pulling telemetry from the underlying SmartMotorController.
 */
public class ArmIOTalonFX implements ArmIO {

  private final Arm arm;
  private final SmartMotorController motor;

  public ArmIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);
    
    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create ArmConfig with the SmartMotorController
    ArmConfig armConfig = new ArmConfig(smc)
        .withLength(Inches.of(18))           // Arm length - used for simulation physics
        .withMass(Pounds.of(5))              // Arm mass - used for simulation physics
        .withHardLimit(Rotations.of(-0.3), Rotations.of(0.3))  // Physical hard stops for sim
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withStartingPosition(Rotations.of(0))
        .withTelemetry("Arm", TelemetryVerbosity.HIGH);
    
    // Step 4: Create Arm mechanism - handles simulation automatically!
    this.arm = new Arm(armConfig);
    
    // Get reference to underlying SmartMotorController for telemetry
    this.motor = arm.getMotor();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations = motor.getMechanismPositionSetpoint()
        .map(a -> a.in(Rotations)).orElse(0.0);
  }

  @Override
  public void setTargetAngle(double rotations) {
    // Use SmartMotorController's setPosition method
    motor.setPosition(Rotations.of(rotations));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }
  
  /** Access the Arm mechanism for command helpers like run() and runTo() */
  public Arm getArm() {
    return arm;
  }
}
```

### Elevator IO Implementation (Works for Real and Sim!)

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Single IO implementation - uses YAMS Elevator mechanism with SmartMotorController telemetry.
 */
public class ElevatorIOTalonFX implements ElevatorIO {

  private final Elevator elevator;
  private final SmartMotorController motor;

  public ElevatorIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);
    
    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
        .withMechanismCircumference(Inches.of(1.5 * Math.PI))  // Pulley circumference
        .withClosedLoopController(10, 0, 0.5)
        .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
        .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create ElevatorConfig with the SmartMotorController
    ElevatorConfig elevatorConfig = new ElevatorConfig(smc)
        .withDrumRadius(Inches.of(0.75))         // Drum radius for pulley
        .withMass(Pounds.of(10))                 // Carriage mass - used for simulation physics
        .withHardLimits(Meters.of(0), Meters.of(1.5))  // Physical hard stops for sim
        .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
        .withStartingHeight(Meters.of(0.5))
        .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
    
    // Step 4: Create Elevator mechanism - handles simulation automatically!
    this.elevator = new Elevator(elevatorConfig);
    
    // Get reference to underlying SmartMotorController for telemetry
    this.motor = elevator.getMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.positionMeters = motor.getMeasurementPosition().in(Meters);
    inputs.velocityMetersPerSec = motor.getMeasurementVelocity().in(MetersPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionMeters = motor.getMeasurementPositionSetpoint()
        .map(d -> d.in(Meters)).orElse(0.0);
  }

  @Override
  public void setTargetHeight(double meters) {
    // Use SmartMotorController's setPosition method with Distance
    motor.setPosition(Meters.of(meters));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }
  
  /** Access the Elevator mechanism for command helpers like run() and runTo() */
  public Elevator getElevator() {
    return elevator;
  }
}
```

### Shooter (FlyWheel) IO Implementation (Works for Real and Sim!)

```java
package frc.robot.subsystems.akit;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Single IO implementation - uses YAMS FlyWheel mechanism with SmartMotorController telemetry.
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final FlyWheel flywheel;
  private final SmartMotorController motor;

  public ShooterIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);
    
    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))  // Direct drive
        .withClosedLoopController(0.5, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create FlyWheelConfig with the SmartMotorController
    FlyWheelConfig flywheelConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))              // Flywheel diameter
        .withMass(Pounds.of(0.5))                // Flywheel mass - used for simulation physics
        .withSoftLimit(RPM.of(0), RPM.of(6000))  // Velocity soft limits
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
    
    // Step 4: Create FlyWheel mechanism - handles simulation automatically!
    this.flywheel = new FlyWheel(flywheelConfig);
    
    // Get reference to underlying SmartMotorController for telemetry
    this.motor = flywheel.getMotor();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetVelocityRotationsPerSec = motor.getMechanismSetpointVelocity()
        .map(v -> v.in(RotationsPerSecond)).orElse(0.0);
  }

  @Override
  public void setTargetVelocity(double rotationsPerSec) {
    // Use SmartMotorController's setVelocity method
    motor.setVelocity(RotationsPerSecond.of(rotationsPerSec));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }
  
  /** Access the FlyWheel mechanism for command helpers like run() and runTo() */
  public FlyWheel getFlyWheel() {
    return flywheel;
  }
}
```

{% hint style="success" %}
**Why This Works**: YAMS's passive simulation is built into every wrapper. When `Robot.isSimulation()` is true, the `TalonFXWrapper` (and all other wrappers) automatically uses the `DCMotor`, gearing, and mass/MOI you provided to simulate physics. You get accurate simulation without writing any simulation-specific code!
{% endhint %}

## Step 3: Create the Subsystem

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
        .until(() -> isNear(Rotations.of(rotations), Rotations.of(0.01)))
        .withName("Arm.goToAngle(" + rotations + ")");
  }

  /** Returns true if the arm is within tolerance of a target position using WPILib's isNear(). */
  public boolean isNear(Angle target, Angle tolerance) {
    return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
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

## Step 4: Wire It Up in RobotContainer

With YAMS's passive simulation, you don't need conditional logic for real vs sim - the same IO class works for both!

```java
package frc.robot;

import frc.robot.subsystems.akit.*;

public class RobotContainer {

  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;
  private final ShooterSubsystem shooter;

  public RobotContainer() {
    // Same IO implementation works for BOTH real robot and simulation!
    // No need for if(RobotBase.isReal()) checks - YAMS handles it automatically
    arm = new ArmSubsystem(new ArmIOTalonFX(arm, 1));
    elevator = new ElevatorSubsystem(new ElevatorIOTalonFX(elevator, 2));
    shooter = new ShooterSubsystem(new ShooterIOTalonFX(shooter, 3));

    configureBindings();
  }

  private void configureBindings() {
    // Commands work identically on real robot and in simulation
    controller.a().whileTrue(arm.setAngle(Rotations.of(0.25)));
    controller.b().whileTrue(arm.setAngle(Rotations.of(-0.25)));
    controller.x().whileTrue(elevator.setHeight(Meters.of(1.0)));
    controller.y().whileTrue(shooter.setSpeed(RotationsPerSecond.of(80)));
  }
}
```

{% hint style="info" %}
**Compare to Traditional AdvantageKit**: Without YAMS, you'd need separate `ArmIOReal` and `ArmIOSim` classes with duplicate configuration. YAMS eliminates this duplication entirely.
{% endhint %}

## Log Replay Mode

For [log replay](https://docs.advantagekit.org/getting-started/what-is-advantagekit), AdvantageKit provides a "replay" mode where inputs are read from a log file instead of hardware. This is the **one case** where you need a separate IO implementation - an empty one that does nothing:

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

In RobotContainer, only check for replay mode:

```java
public RobotContainer() {
  if (Constants.currentMode == Mode.REPLAY) {
    // Replay mode: empty IO, AdvantageKit injects logged values
    arm = new ArmSubsystem(new ArmIOReplay());
    elevator = new ElevatorSubsystem(new ElevatorIOReplay());
    shooter = new ShooterSubsystem(new ShooterIOReplay());
  } else {
    // Real AND Sim: same IO class works for both!
    arm = new ArmSubsystem(new ArmIOTalonFX(arm, 1));
    elevator = new ElevatorSubsystem(new ElevatorIOTalonFX(elevator, 2));
    shooter = new ShooterSubsystem(new ShooterIOTalonFX(shooter, 3));
  }
}
```

{% hint style="success" %}
**Summary**: With YAMS + AdvantageKit, you only need **2 IO classes** per mechanism:
1. **One real IO class** (e.g., `ArmIOTalonFX`) - works for both real robot and simulation
2. **One empty replay IO class** (e.g., `ArmIOReplay`) - for log replay only

Compare this to traditional AdvantageKit which requires **3 IO classes** (Real, Sim, Replay).
{% endhint %}

## Pattern Summary: Mechanism Classes with SmartMotorController Telemetry

The examples above demonstrate the recommended pattern for using YAMS with AdvantageKit:

1. **Create Mechanism classes** (`Arm`, `Elevator`, `FlyWheel`, etc.) for high-level control
2. **Get the underlying `SmartMotorController`** via `mechanism.getSmartMotorController()`
3. **Pull telemetry from `SmartMotorController`** for AdvantageKit logging
4. **Use Mechanism methods** like `setMechanismPositionSetpoint()` for control
5. **Optionally expose the Mechanism** via a getter for command helpers like `run()` and `runTo()`

This pattern gives you the best of both worlds:
- **Mechanism classes** provide convenient configuration and command helpers
- **SmartMotorController** provides detailed telemetry for AdvantageKit logging
- **Passive simulation** works automatically in both

{% hint style="info" %}
See the [YAMS example code](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple_robot/src/main/java/frc/robot/subsystems/akit) for complete examples of using YAMS Mechanism classes with AdvantageKit's IO pattern.
{% endhint %}

### Using WPILib's isNear() for Tolerance Checking

When checking if a mechanism is at its target, use the `isNear()` method from WPILib's unit classes (`Angle`, `Distance`, `AngularVelocity`):

```java
// In your subsystem, check if position is near target
Angle currentPosition = Rotations.of(inputs.positionRotations);
Angle targetPosition = Rotations.of(targetRotations);
Angle tolerance = Rotations.of(0.01);

boolean atTarget = currentPosition.isNear(targetPosition, tolerance);
```

This approach works with any unit type and handles conversions automatically.

## Benefits of YAMS + AdvantageKit

| Feature | Benefit |
|---------|---------|
| **No Duplicate IO Classes** | Single IO class works for real robot AND simulation |
| **Deterministic Replay** | Debug issues from real matches using log replay |
| **Automatic Physics** | YAMS uses DCMotor, gearing, and mass/MOI for accurate sim |
| **Less Boilerplate** | 2 IO classes instead of 3 per mechanism |
| **Consistent Behavior** | Same code paths execute in real and sim modes |
| **Flexible Hardware** | Swap vendors by changing one IO class |

## Further Reading

- [AdvantageKit Documentation](https://docs.advantagekit.org/)
- [What is AdvantageKit?](https://docs.advantagekit.org/getting-started/what-is-advantagekit)
- [IO Interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces)
- [Template Projects](https://docs.advantagekit.org/getting-started/template-projects)
- [YAMS Example Code](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple_robot/src/main/java/frc/robot/subsystems/akit)
