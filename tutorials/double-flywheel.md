---
description: We will learn how to program a Double FlyWheel shooter!
---

# Double FlyWheel

{% hint style="danger" %}
Mechanism classes are meant to be used with "tightly coupled" mechanisms where the Mechanism has 1 or more motor controlling it on a connected shaft, gearbox, or other linkage.

**IF** your mechanism is "loosely coupled", you **CAN** still use YAMS. **HOWEVER** you have to create and control the `SmartMotorController` directly as shown in [how-do-i-control-a-mechanism-without-a-mechanism-class.md](../how-to/how-do-i-control-a-mechanism-without-a-mechanism-class.md "mention") OR use `SmartMotorControllerConfig.withLooselyCoupledFollowers(SmartMotorController...)`
{% endhint %}

## Intro

At the end of this tutorial you will have a **Double FlyWheel** shooter with two independently controlled flywheels that will work in both real life and simulation with the same code! Double flywheels are commonly used in FRC games where spin control on the game piece is important - one wheel spinning faster than the other can add backspin or topspin to shots.

## What is a Double FlyWheel?

A double flywheel is a shooter mechanism with two separate flywheels, typically arranged as:

* **Upper FlyWheel** - Controls the top surface of the game piece
* **Lower FlyWheel** - Controls the bottom surface of the game piece

By running these at different speeds, you can:

* Add **backspin** for lob shots (upper faster than lower)
* Add **topspin** for line drive shots (lower faster than upper)
* Achieve **neutral spin** for consistent shots (equal speeds)

## Details

This `DoubleFlyWheel` will use the following hardware specs and control details:

* Two `TalonFX` (Kraken X60) motors controlling each flywheel independently
* `3:1` GearBox on each flywheel
* 4 inch diameter, 1 pound flywheels
* Pressing `A` will set both flywheels to 3000 RPM (neutral spin)
* Pressing `B` will set upper to 3500 RPM, lower to 2500 RPM (backspin)
* Pressing `X` will set upper to 2500 RPM, lower to 3500 RPM (topspin)
* Pressing `Y` will stop both flywheels

## Lets create a WPILib Command-Based Project!

{% hint style="success" %}
IF you already have a project and know how to place the `DoubleFlyWheel` mechanism into your own subsystem please skip to [#install-yams](double-flywheel.md#install-yams "mention")
{% endhint %}

### Setup our Command-Based Project

Follow [WPILib's tutorial](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-test-drivetrain-program-cpp-java-python.html) on how to create a Command-Based project.

Bring up the Visual Studio Code command palette with <kbd>Ctrl+Shift+P</kbd>. Then, type "WPILib" into the prompt and select "Create a new project".

1. Click on **Select a project type (Example or Template)**
2. Select **Template** then **Java** then **Command Robot**
3. Click on **Select a new project folder** and select a folder to store your robot project in.
4. Fill in **Project Name** with the name of your robot code project.
5. Enter your **Team Number**
6. Be sure to check **Enable Desktop Support** so we can run simulations!

### Install YAMS!

Click on the **WPILib logo** on the **left** pane. Scroll down to **Yet Another Mechanism System** and click **Install**!

## Let's make a Double FlyWheel!

{% stepper %}
{% step %}
#### Create our Subsystem with two `SmartMotorControllerConfig`s

We need separate configurations for each flywheel since they will be controlled independently.

```java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class DoubleFlywheelSubsystem extends SubsystemBase {

  // Upper FlyWheel Motor Config
  private SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // PID Constants for velocity control
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      // Feedforward Constants - helps track changing RPM goals
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      // Telemetry name and verbosity level
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft (3:1 reduction)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      // Motor properties
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Lower FlyWheel Motor Config  
  private SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(true) // Often inverted relative to upper for proper game piece direction
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  /** Creates a new DoubleFlywheelSubsystem. */
  public DoubleFlywheelSubsystem() {}
}
```
{% endstep %}

{% step %}
#### Create our motor controllers

Create the vendor motor controller objects for each flywheel.

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class DoubleFlywheelSubsystem extends SubsystemBase {

  // Upper FlyWheel Motor Config
  private SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Lower FlyWheel Motor Config  
  private SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller objects
  private TalonFX upperTalon = new TalonFX(1);
  private TalonFX lowerTalon = new TalonFX(2);

  // Create our SmartMotorControllers with Kraken X60 motors
  private SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
  private SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

  /** Creates a new DoubleFlywheelSubsystem. */
  public DoubleFlywheelSubsystem() {}
}
```
{% endstep %}

{% step %}
#### Create and Configure our `FlyWheel` mechanisms

Each flywheel gets its own `FlyWheel` mechanism for intuitive control.

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class DoubleFlywheelSubsystem extends SubsystemBase {

  // Upper FlyWheel Motor Config
  private SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Lower FlyWheel Motor Config  
  private SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller objects
  private TalonFX upperTalon = new TalonFX(1);
  private TalonFX lowerTalon = new TalonFX(2);

  // Create our SmartMotorControllers with Kraken X60 motors
  private SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
  private SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

  // Upper FlyWheel Mechanism Config
  private final FlyWheelConfig upperFlywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("UpperFlyWheelMech", TelemetryVerbosity.HIGH);

  // Lower FlyWheel Mechanism Config
  private final FlyWheelConfig lowerFlywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("LowerFlyWheelMech", TelemetryVerbosity.HIGH);

  // FlyWheel Mechanisms
  private FlyWheel upperFlywheel = new FlyWheel(upperFlywheelConfig, upperMotor);
  private FlyWheel lowerFlywheel = new FlyWheel(lowerFlywheelConfig, lowerMotor);

  /** Creates a new DoubleFlywheelSubsystem. */
  public DoubleFlywheelSubsystem() {}

  @Override
  public void periodic() {
    // Update telemetry for both flywheels
    upperFlywheel.updateTelemetry();
    lowerFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Run simulation for both flywheels
    upperFlywheel.simIterate();
    lowerFlywheel.simIterate();
  }
}
```
{% endstep %}

{% step %}
#### Create `Command`s for our Double FlyWheel

Now we create command factory methods for different shot types.

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class DoubleFlywheelSubsystem extends SubsystemBase {

  // Upper FlyWheel Motor Config
  private SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Lower FlyWheel Motor Config  
  private SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller objects
  private TalonFX upperTalon = new TalonFX(1);
  private TalonFX lowerTalon = new TalonFX(2);

  // Create our SmartMotorControllers with Kraken X60 motors
  private SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
  private SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

  // Upper FlyWheel Mechanism Config
  private final FlyWheelConfig upperFlywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("UpperFlyWheelMech", TelemetryVerbosity.HIGH);

  // Lower FlyWheel Mechanism Config
  private final FlyWheelConfig lowerFlywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("LowerFlyWheelMech", TelemetryVerbosity.HIGH);

  // FlyWheel Mechanisms
  private FlyWheel upperFlywheel = new FlyWheel(upperFlywheelConfig, upperMotor);
  private FlyWheel lowerFlywheel = new FlyWheel(lowerFlywheelConfig, lowerMotor);

  /** Creates a new DoubleFlywheelSubsystem. */
  public DoubleFlywheelSubsystem() {}

  /**
   * Set both flywheel speeds independently.
   *
   * @param upperSpeed Upper flywheel target speed
   * @param lowerSpeed Lower flywheel target speed
   * @return Command to set both flywheel speeds
   */
  public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
    return Commands.parallel(
        upperFlywheel.run(upperSpeed),
        lowerFlywheel.run(lowerSpeed)
    );
  }

  /**
   * Neutral spin shot - both wheels at same speed.
   *
   * @return Command for neutral spin shot
   */
  public Command neutralSpinShot() {
    return setFlywheelSpeeds(RPM.of(3000), RPM.of(3000));
  }

  /**
   * Backspin shot - upper wheel faster for lob shots.
   *
   * @return Command for backspin shot
   */
  public Command backspinShot() {
    return setFlywheelSpeeds(RPM.of(3500), RPM.of(2500));
  }

  /**
   * Topspin shot - lower wheel faster for line drives.
   *
   * @return Command for topspin shot
   */
  public Command topspinShot() {
    return setFlywheelSpeeds(RPM.of(2500), RPM.of(3500));
  }

  /**
   * Stop both flywheels.
   *
   * @return Command to stop flywheels
   */
  public Command stopFlywheels() {
    return setFlywheelSpeeds(RPM.of(0), RPM.of(0));
  }

  /**
   * Check if both flywheels are near their target velocities.
   *
   * @param tolerance Acceptable velocity error tolerance
   * @return true if both flywheels are within tolerance of their target
   */
  public boolean atTargetVelocity(AngularVelocity tolerance) {
    return upperFlywheel.isNear(tolerance) && lowerFlywheel.isNear(tolerance);
  }

  /**
   * Get the current upper flywheel velocity.
   *
   * @return Upper flywheel velocity
   */
  public AngularVelocity getUpperVelocity() {
    return upperFlywheel.getVelocity();
  }

  /**
   * Get the current lower flywheel velocity.
   *
   * @return Lower flywheel velocity
   */
  public AngularVelocity getLowerVelocity() {
    return lowerFlywheel.getVelocity();
  }

  @Override
  public void periodic() {
    upperFlywheel.updateTelemetry();
    lowerFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    upperFlywheel.simIterate();
    lowerFlywheel.simIterate();
  }
}
```
{% endstep %}

{% step %}
#### Configure controller bindings in `RobotContainer`

Wire up the controller buttons to our shot types.

```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DoubleFlywheelSubsystem;

public class RobotContainer {

  private final DoubleFlywheelSubsystem m_doubleFlywheel = new DoubleFlywheelSubsystem();
  private final CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // A button - Neutral spin shot (both wheels at 3000 RPM)
    m_controller.a().onTrue(m_doubleFlywheel.neutralSpinShot());
    
    // B button - Backspin shot (upper faster)
    m_controller.b().onTrue(m_doubleFlywheel.backspinShot());
    
    // X button - Topspin shot (lower faster)
    m_controller.x().onTrue(m_doubleFlywheel.topspinShot());
    
    // Y button - Stop flywheels
    m_controller.y().onTrue(m_doubleFlywheel.stopFlywheels());
  }
}
```
{% endstep %}
{% endstepper %}

## Using Loosely Coupled Followers

If your double flywheel design has both motors mechanically linked (same shaft), you can use loosely coupled followers instead of separate mechanisms:

```java
SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
    // ... config options
    ;

SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
    // ... config options
    .withLooselyCoupledFollowers(lowerMotor); // Lower follows upper's setpoints

SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
```

{% hint style="warning" %}
Loosely coupled followers only forward velocity and position requests, **NOT** DutyCycle or Voltage requests. Use separate mechanisms when you need fully independent control.
{% endhint %}

## Advanced: Dynamic Spin Control

For more advanced shot control, you can create a method that calculates spin ratios:

```java
/**
 * Set flywheel speeds based on spin ratio.
 *
 * @param baseSpeed The average speed of both wheels
 * @param spinRatio Ratio from -1.0 (max topspin) to 1.0 (max backspin)
 * @return Command to set calculated flywheel speeds
 */
public Command setSpinShot(AngularVelocity baseSpeed, double spinRatio) {
  double ratio = Math.max(-1.0, Math.min(1.0, spinRatio)); // Clamp to [-1, 1]
  double upperMultiplier = 1.0 + (ratio * 0.25); // +/- 25% variation
  double lowerMultiplier = 1.0 - (ratio * 0.25);
  
  return setFlywheelSpeeds(
      RPM.of(baseSpeed.in(RPM) * upperMultiplier),
      RPM.of(baseSpeed.in(RPM) * lowerMultiplier)
  );
}
```

## Summary

You now have a fully functional double flywheel shooter with:

* Independent velocity control for upper and lower flywheels
* Preset shot types (neutral, backspin, topspin)
* Full simulation support
* Telemetry for tuning both flywheels

The key differences from a single flywheel are:

1. Two separate `SmartMotorControllerConfig` instances
2. Two separate `SmartMotorController` instances
3. Two separate `FlyWheel` mechanism instances
4. Commands that coordinate both flywheels using `Commands.parallel()`
