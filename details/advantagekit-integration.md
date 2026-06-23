---
icon: database
---

# AdvantageKit Integration

YAMS integrates seamlessly with [AdvantageKit](https://docs.advantagekit.org/), Team 6328's logging and replay framework. This guide shows how to use YAMS mechanisms with AdvantageKit's IO layer pattern for deterministic log replay.

## What is AdvantageKit?

[AdvantageKit](https://docs.advantagekit.org/getting-started/what-is-advantagekit) is a logging framework that records **all inputs** to your robot code, enabling deterministic replay in simulation. Unlike traditional logging that captures specific values, AdvantageKit captures everything flowing into your code, allowing you to:

* Replay matches exactly as they happened
* Add new logged fields after the fact
* Test code changes against real match data
* Debug issues that only occurred on the real robot

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

* **One IO class** handles both real robot and simulation
* **No duplicate code** - configuration is written once
* **No SimWrapper needed** - real hardware wrappers simulate themselves
* **Physics are automatic** - based on your DCMotor, gearing, and mass/MOI configuration

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

  /** Update the sim telemetry */
  default void simIterate() {}

  /** Update the telemtry */
  default void updateTelemetry() {}
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

  /** Update the sim telemetry */
  default void simIterate() {}

  /** Update the telemtry */
  default void updateTelemetry() {}
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

  /** Update the sim telemetry */
  default void simIterate() {}

  /** Update the telemtry */
  default void updateTelemetry() {}
}
```

{% hint style="info" %}
The `@AutoLog` annotation from AdvantageKit generates a class (e.g., `ArmIOInputsAutoLogged`) that handles serialization for log replay. See [AdvantageKit AutoLog documentation](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces#autolog) for details.

If the @AutoLog fails to generate the coresponding AutoLogged class, check your build.gradle. In the dependencies section you should have

### Build.gradle

```java

def akitJson = new groovy.json.JsonSlurper().parseText(new File(projectDir.getAbsolutePath() + "/vendordeps/AdvantageKit.json").text)
    annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"

```  

Comment or delete out this line 'annotationProcessor wpi.java.deps.wpilibAnnotations()' and clean the project with a .gradle clean (windows) or ./gradlew clean (linux) and then build
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
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create ArmConfig with the SmartMotorController
    ArmConfig armConfig = new ArmConfig(smc)
        .withLength(Inches.of(18))           // Arm length - used for simulation physics
        .withMass(Pounds.of(5))              // Arm mass - used for simulation physics
        .withHardLimit(Rotations.of(-0.3), Rotations.of(0.3))  // Physical hard stops for sim
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

  @Override
  public void simIterate() {
    arm.simIterate();
  }

  @Override
  public void updateTelemetry() {
    arm.updateTelemetry();
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
        .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
        .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
        .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create ElevatorConfig with the SmartMotorController
    ElevatorConfig elevatorConfig = new ElevatorConfig(smc)
        .withDrumRadius(Inches.of(0.75))         // Drum radius for pulley
        .withMass(Pounds.of(10))                 // Carriage mass - used for simulation physics
        .withHardLimits(Meters.of(0), Meters.of(1.5))  // Physical hard stops for sim
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

  @Override
  public void simIterate() {
    elevator.simIterate();
  }

  @Override
  public void updateTelemetry() {
    elevator.updateTelemetry();
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
        .withSoftLimit(RPM.of(0), RPM.of(6000))  // Velocity soft limits
        .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));
    
    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    
    // Step 3: Create FlyWheelConfig with the SmartMotorController
    FlyWheelConfig flywheelConfig = new FlyWheelConfig(smc)
        .withDiameter(Inches.of(4))              // Flywheel diameter
        .withMass(Pounds.of(0.5))                // Flywheel mass - used for simulation physics
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

  @Override
  public void simIterate() {
    flywheel.simIterate();
  }

  @Override
  public void updateTelemetry() {
    flywheel.updateTelemetry();
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
    // Update the arms telemetry
    io.updateTelemetry();
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  @Override
  public void simulationPeriodic() {
    io.simIterate();
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

* **Mechanism classes** provide convenient configuration and command helpers
* **SmartMotorController** provides detailed telemetry for AdvantageKit logging
* **Passive simulation** works automatically in both

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

## Swerve Drive with AdvantageKit

YAMS swerve integrates seamlessly with AdvantageKit. Here's a complete example based on the [official YAMS AdvantageKit swerve example](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/advantage_kit/src/main/java/frc/robot/subsystems/SwerveSubsystem.java):

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase {

  private AngularVelocity maxAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity maxLinearVelocity = MetersPerSecond.of(4);
  private Supplier<Angle> gyroAngleSupplier;
  private SwerveDriveConfig config;

  /** AdvantageKit inputs for the swerve drive */
  @AutoLog
  public static class SwerveInputs {
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    public SwerveModuleState[] states = new SwerveModuleState[4];
    public Angle gyroRotation = Degrees.of(0);
    public ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    public Pose2d estimatedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
  private final SwerveDrive drive;

  public SwerveModule createModule(SparkMax driveMotor, SparkMax azimuthMotor, 
      CANcoder absoluteEncoder, String moduleName, Translation2d location) {
    
    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));
    
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
    
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
    
    SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);
    
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);
    
    return new SwerveModule(moduleConfig);
  }

  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier x, DoubleSupplier y, 
      DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot)
        .withMaximumAngularVelocity(maxAngularVelocity)
        .withMaximumLinearVelocity(maxLinearVelocity)
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl();
  }

  public SwerveSubsystem() {
    Pigeon2 gyro = new Pigeon2(14);
    gyroAngleSupplier = gyro.getYaw().asSupplier();

    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
        new SparkMax(2, MotorType.kBrushless), new CANcoder(3),
        "frontleft", new Translation2d(Inches.of(12), Inches.of(12)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
        new SparkMax(5, MotorType.kBrushless), new CANcoder(6),
        "frontright", new Translation2d(Inches.of(12), Inches.of(-12)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
        new SparkMax(8, MotorType.kBrushless), new CANcoder(9),
        "backleft", new Translation2d(Inches.of(-12), Inches.of(12)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
        new SparkMax(11, MotorType.kBrushless), new CANcoder(12),
        "backright", new Translation2d(Inches.of(-12), Inches.of(-12)));

    config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(() -> getGyroAngle().getMeasure())
        .withStartingPose(swerveInputs.estimatedPose)
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    
    drive = new SwerveDrive(config);
  }

  private Rotation2d getGyroAngle() {
    return new Rotation2d(swerveInputs.gyroRotation);
  }

  private void updateInputs() {
    swerveInputs.estimatedPose = drive.getPose();
    swerveInputs.states = drive.getModuleStates();
    swerveInputs.positions = drive.getModulePositions();
    swerveInputs.robotRelativeSpeeds = drive.getRobotRelativeSpeed();
    swerveInputs.gyroRotation = gyroAngleSupplier.get();
  }

  public Command setRobotRelativeChassisSpeeds(Supplier<ChassisSpeeds> speedsSupplier) {
    return run(() -> {
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speedsSupplier.get());
      SwerveModuleState[] states = drive.getStateFromRobotRelativeChassisSpeeds(speedsSupplier.get());
      Logger.recordOutput("Swerve/DesiredStates", states);
      drive.setSwerveModuleStates(states);
    }).withName("Set Robot Relative Chassis Speeds");
  }

  public Command driveToPose(Pose2d pose) {
    return drive.driveToPose(pose);
  }

  public Command lock() {
    return run(drive::lockPose);
  }

  public Pose2d getPose() { return swerveInputs.estimatedPose; }
  public ChassisSpeeds getRobotRelativeSpeeds() { return swerveInputs.robotRelativeSpeeds; }

  @Override
  public void periodic() {
    drive.updateTelemetry();  // Updates pose estimator - call BEFORE updateInputs
    updateInputs();
    Logger.processInputs("Swerve", swerveInputs);
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }
}
```

### Key Points for Swerve with AdvantageKit

1. **Create an `@AutoLog` inputs class** with module states, positions, gyro, and pose
2. **Call `updateTelemetry()` BEFORE `updateInputs()`** - the pose estimator and internal state updates in `updateTelemetry()`
3. **Log desired states and speeds** using `Logger.recordOutput()` for replay debugging
4. **Call `simIterate()` in `simulationPeriodic()`** for physics simulation

{% hint style="warning" %}
**Important: Replay Data Scope**

During AdvantageKit log replay, **only data inside the `@AutoLog` inputs classes is mutated**. YAMS's built-in telemetry (published via `withTelemetry()`) is NOT affected by replay - it will show live simulation values, not replayed values.

This means:

* **Inputs you define** (in `SwerveInputs`, `ArmInputs`, etc.) - Replayed correctly
* **YAMS telemetry** (from `.withTelemetry("Arm")`) - Shows live sim values during replay

To ensure accurate replay debugging, always pull the data you need into your `@AutoLog` inputs class rather than relying on YAMS telemetry during replay.
{% endhint %}

## Alternative Pattern: Direct Subsystem (No IO Layer)

For simpler projects where replay isn't needed, you can use YAMS mechanisms directly as shown in the [official examples](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/advantage_kit/src/main/java/frc/robot/subsystems):

```java
public class ArmSubsystem extends SubsystemBase {

  @AutoLog
  public static class ArmInputs {
    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);
  }

  private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();
  private final SmartMotorController armSMC;
  private final Arm arm;

  public ArmSubsystem() {
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(18, 0, 0.2)
        .withTrapezoidalProfile(DegreesPerSecond.of(458), DegreesPerSecondPerSecond.of(688))
        .withFeedforward(new ArmFeedforward(-0.1, 1.2, 0, 0))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(120));

    armSMC = new TalonFXWrapper(new TalonFX(40), DCMotor.getFalcon500(1), smcConfig);

    ArmConfig armCfg = new ArmConfig(armSMC)
        .withHardLimit(Degrees.of(-25), Degrees.of(141))
        .withStartingPosition(Degrees.of(141))
        .withLength(Feet.of(14.0 / 12))
        .withMOI(0.1055)
        .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    arm = new Arm(armCfg);
  }

  public void updateInputs() {
    armInputs.pivotPosition = arm.getAngle();
    armInputs.pivotVelocity = armSMC.getMechanismVelocity();
    armInputs.pivotAppliedVolts = armSMC.getVoltage();
    armInputs.pivotCurrent = armSMC.getStatorCurrent();
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();  // Always call BEFORE updateInputs
    updateInputs();
    Logger.processInputs("Arm", armInputs);
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
```

This pattern is simpler and **does support log replay**, but with a key difference: during replay, the full YAMS simulation runs alongside the logged data. This means:

* **Pro**: You get full simulation behavior during replay, which can help debug physics-related issues
* **Con**: Replay requires more processing power since every `SmartMotorController` and Mechanism simulates its physics

Choose the IO layer pattern if you need lightweight replay; choose this direct pattern if you prefer simplicity and don't mind the extra processing during replay.

## Benefits of YAMS + AdvantageKit

| Feature                     | Benefit                                                   |
| --------------------------- | --------------------------------------------------------- |
| **No Duplicate IO Classes** | Single IO class works for real robot AND simulation       |
| **Deterministic Replay**    | Debug issues from real matches using log replay           |
| **Automatic Physics**       | YAMS uses DCMotor, gearing, and mass/MOI for accurate sim |
| **Less Boilerplate**        | 2 IO classes instead of 3 per mechanism                   |
| **Consistent Behavior**     | Same code paths execute in real and sim modes             |
| **Flexible Hardware**       | Swap vendors by changing one IO class                     |

## Further Reading

* [AdvantageKit Documentation](https://docs.advantagekit.org/)
* [What is AdvantageKit?](https://docs.advantagekit.org/getting-started/what-is-advantagekit)
* [IO Interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces)
* [Template Projects](https://docs.advantagekit.org/getting-started/template-projects)
* [YAMS Example Code](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple_robot/src/main/java/frc/robot/subsystems/akit)
