---
description: We will learn how to program an Elevator!
---

# Elevator

{% hint style="danger" %}
Mechanism classes are meant to be used with "tightly coupled" mechanisms where the Mechanism has 1 or more motor controlling it on a connected shaft, gearbox, or other linkage.&#x20;

**IF** your mechanism is "loosely coupled", you **CAN** still use YAMS. **HOWEVER** you have to create and control the `SmartMotorController` directly as shown in [how-do-i-control-a-mechanism-without-a-mechanism-class.md](../how-to/how-do-i-control-a-mechanism-without-a-mechanism-class.md "mention") OR use `SmartMotorControllerConfig.withLooselyCoupledFollowers(SmartMotorController...)`
{% endhint %}

## Intro

At the end of this tutorial you will have an `Elevator` that will work in both real life and simulation with the same code!

{% columns %}
{% column %}
### Simulation

<figure><img src="../.gitbook/assets/Robot Simulation 2025-08-30 18-27-07.gif" alt=""><figcaption></figcaption></figure>

<figure><img src="../.gitbook/assets/Robot Simulation 2025-08-30 18-37-59.gif" alt=""><figcaption></figcaption></figure>
{% endcolumn %}

{% column %}
### Real Life

{% embed url="https://www.youtube.com/shorts/XKmsW8-85Dg" %}
{% endcolumn %}
{% endcolumns %}

## Details

This `Elevator` will be using the following hardware specs and control details

* `SparkMax` controlling the `Elevator`&#x20;
* `12:1` GearBox on the `Elevator`&#x20;
* Pressing `A` will make the `Elevator` go to 0.5m
* Pressing `B` will make the `Elevator` go to 1
* Pressing `X` will make the `Elevator` go up.
* Pressing `Y` will make the `Elevator` go down.

## Lets create a WPILib Command-Based Project!

{% hint style="success" %}
IF you already have a project and know how to place the `Elevator` mechanism into your own subsystem please skip to [#install-yams](elevator.md#install-yams "mention")
{% endhint %}

### Setup our Command-Based Project

Here we will follow [WPILib's tutorial ](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-test-drivetrain-program-cpp-java-python.html)on how to create a Command-Based project.

Bring up the Visual Studio Code command palette with <kbd>Ctrl+Shift+P</kbd>. Then, type “WPILib” into the prompt. Since all WPILib commands start with “WPILib”, this will bring up the list of WPILib-specific VS Code commands. Now, select the “Create a new project” command:

<figure><img src="../.gitbook/assets/image (3).png" alt=""><figcaption></figcaption></figure>

This will bring up the “New Project Creator Window:”

<figure><img src="../.gitbook/assets/image (7).png" alt=""><figcaption></figcaption></figure>

1. Click on **Select a project type (Example or Template)**
2. Select **Template** then **Java** then **Command Robot**
3. Click on **Select a new project folder** and select a folder to store your robot project in.
4. Fill in **Project Name** with the name of your robot code project.
5. Enter your **Team Number** in so you can deploy to your robot.
6. Be sure to check **Enable Desktop Support** so we can run simulations!

If you followed these instructions it should look something like whats filled out below.

<figure><img src="../.gitbook/assets/image (6).png" alt=""><figcaption></figcaption></figure>

Congratulations! You now have a Command Based robot project!

<figure><img src="../.gitbook/assets/image (10).png" alt=""><figcaption></figcaption></figure>

### Install YAMS!

Click on the **WPILib logo** on the **left** pane. Scroll down to **Yet Another Mechanism System** and click **Install**!

<figure><img src="../.gitbook/assets/RobotContainer.java - simple_robot - Visual Studio Code 9_2_2025 3_40_21 PM.png" alt=""><figcaption></figcaption></figure>

Congratulations you have now installed YAMS! :tada:

## Lets make an Elevator move!

{% stepper %}
{% step %}
### Create a `SmartMotorControllerConfig`

We are going to start by configuring out motor controller.

<pre class="language-java" data-title="ExampleSubsystem.java" data-line-numbers data-full-width="true"><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<strong>import static edu.wpi.first.units.Units.Amps;
</strong><strong>import static edu.wpi.first.units.Units.MetersPerSecond;
</strong><strong>import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
</strong><strong>import static edu.wpi.first.units.Units.Seconds;
</strong>
<strong>import edu.wpi.first.math.controller.ElevatorFeedforward;
</strong>import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<strong>import yams.mechanisms.SmartMechanism;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
</strong><strong>import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
</strong>
public class ExampleSubsystem extends SubsystemBase {

<strong>  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
</strong><strong>  .withControlMode(ControlMode.CLOSED_LOOP)
</strong><strong>  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
</strong><strong>  .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
</strong><strong>  // Feedback Constants (PID Constants)
</strong><strong>  .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
</strong><strong>  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
</strong><strong>  // Feedforward Constants
</strong><strong>  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
</strong><strong>  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
</strong><strong>  // Telemetry name and verbosity level
</strong><strong>  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
</strong><strong>  // Gearing from the motor rotor to final shaft.
</strong><strong>  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
</strong><strong>  // You could also use .withGearing(12) which does the same thing.
</strong><strong>  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
</strong><strong>  // Motor properties to prevent over currenting.
</strong><strong>  .withMotorInverted(false)
</strong><strong>  .withIdleMode(MotorMode.BRAKE)
</strong><strong>  .withStatorCurrentLimit(Amps.of(40))
</strong><strong>  .withClosedLoopRampRate(Seconds.of(0.25))
</strong><strong>  .withOpenLoopRampRate(Seconds.of(0.25));
</strong>
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

</code></pre>
{% endstep %}

{% step %}
### Create our motor controller

To control our `Elevator` motor we will create the vendor motor controller object.

First we install `REVLib` by clicking on the WPILib logo in the left bar.

<figure><img src="../.gitbook/assets/image (14).png" alt=""><figcaption></figcaption></figure>

Now we add it to the `ExampleSubsystem.java`

<pre class="language-java" data-title="ExampleSubsystem.java" data-line-numbers><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

<strong>import com.revrobotics.spark.SparkLowLevel.MotorType;
</strong><strong>import com.revrobotics.spark.SparkMax;
</strong>
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ExampleSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

<strong>  // Vendor motor controller object
</strong><strong>  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);
</strong>

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
</code></pre>
{% endstep %}

{% step %}
### Create our `SmartMotorController`

Our `SmartMotorController` will easily configure and interface with the vendor motor controller object.

<pre class="language-java" data-title="ExampleSubsystem.java" data-line-numbers><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
<strong>import edu.wpi.first.math.system.plant.DCMotor;
</strong>import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
<strong>import yams.motorcontrollers.SmartMotorController;
</strong>import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
<strong>import yams.motorcontrollers.local.SparkWrapper;
</strong>
public class ExampleSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

<strong>  // Create our SmartMotorController from our Spark and config with the NEO.
</strong><strong>  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
</strong>
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

</code></pre>
{% endstep %}

{% step %}
### Create and Configure our `Elevator`

Our `Elevator` will easily configure the `SmartMotorController` and create a simple and intuitive interface.

<pre class="language-java" data-title="ExampleSubsystem.java" data-line-numbers><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
<strong>import static edu.wpi.first.units.Units.Feet;
</strong><strong>import static edu.wpi.first.units.Units.Pounds;
</strong>import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
<strong>import yams.mechanisms.config.ElevatorConfig;
</strong><strong>import yams.mechanisms.positional.Elevator;
</strong>import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ExampleSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

<strong>  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
</strong><strong>      .withStartingHeight(Meters.of(0.5))
</strong><strong>      .withHardLimits(Meters.of(0), Meters.of(3))
</strong><strong>      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
</strong><strong>      .withMass(Pounds.of(16));
</strong>
<strong>  // Elevator Mechanism
</strong><strong>  private Elevator elevator = new Elevator(elevconfig);
</strong>
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
<strong>    // This method will be called once per scheduler run
</strong><strong>    elevator.updateTelemetry();
</strong>  }

  @Override
  public void simulationPeriodic() {
<strong>    // This method will be called once per scheduler run during simulation
</strong><strong>    elevator.simIterate();
</strong>  }
}
</code></pre>
{% endstep %}

{% step %}
### Create `Command`s with our `Elevator`

We use the `Elevator` class as a interface to create commands!

<pre class="language-java" data-title="ExampleSubsystem.java" data-line-numbers><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
<strong>import static edu.wpi.first.units.Units.Second;
</strong><strong>import static edu.wpi.first.units.Units.Seconds;
</strong><strong>import static edu.wpi.first.units.Units.Volts;
</strong>
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
<strong>import edu.wpi.first.units.measure.Distance;
</strong>import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ExampleSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
    .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
    // Feedback Constants (PID Constants)
    .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    // Feedforward Constants
    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    // Telemetry name and verbosity level
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(4, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

<strong>  /**
</strong><strong>   * Set the height of the elevator and does not end the command when reached.
</strong><strong>   * @param angle Distance to go to.
</strong><strong>   * @return a Command
</strong><strong>   */
</strong><strong>  public Command setHeight(Distance height) { return elevator.run(height);}
</strong>  
<strong>  /**
</strong><strong>   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
</strong><strong>   * @param height Distance to go to.
</strong><strong>   * @param tolerance Distance tolerance for completion.
</strong><strong>   * @return A Command
</strong><strong>   */
</strong><strong>  public Command setHeightAndStop(Distance height, Distance tolerance) { return elevator.runTo(height, tolerance);}
</strong>  
<strong>  /**
</strong><strong>   * Set the elevators closed loop controller setpoint.
</strong><strong>   * @param angle Distance to go to.
</strong><strong>   */
</strong><strong>  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}
</strong>
<strong>  /**
</strong><strong>   * Move the elevator up and down.
</strong><strong>   * @param dutycycle [-1, 1] speed to set the elevator too.
</strong><strong>   */
</strong><strong>  public Command set(double dutycycle) { return elevator.set(dutycycle);}
</strong>
<strong>  /**
</strong><strong>   * Run sysId on the {@link Elevator}
</strong><strong>   */
</strong><strong>  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}
</strong>
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}

</code></pre>
{% endstep %}

{% step %}
### Bind buttons to our `Elevator`

We bind buttons to use the `Command` from our `Elevator`

<pre class="language-java" data-title="RobotContainer.java" data-line-numbers><code class="lang-java">// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

<strong>import static edu.wpi.first.units.Units.Meters;
</strong>
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

<strong>    // Set the default command to force the elevator to go to 0.
</strong><strong>    m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.setHeight(Meters.of(0)));
</strong>  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
<strong>    // Schedule `setHeight` when the Xbox controller's B button is pressed,
</strong><strong>    // cancelling on release.
</strong><strong>    m_driverController.a().whileTrue(m_exampleSubsystem.setHeight(Meters.of(0.5)));
</strong><strong>    m_driverController.b().whileTrue(m_exampleSubsystem.setHeight(Meters.of(1)));
</strong><strong>    // Schedule `set` when the Xbox controller's B button is pressed,
</strong><strong>    // cancelling on release.
</strong><strong>    m_driverController.x().whileTrue(m_exampleSubsystem.set(0.3));
</strong><strong>    m_driverController.y().whileTrue(m_exampleSubsystem.set(-0.3));
</strong>
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

</code></pre>
{% endstep %}

{% step %}
### Simulate our Elevator!

We can use our `Elevator` in simulation, with the exact same code that will control the real robot!

<figure><img src="../.gitbook/assets/image (16).png" alt=""><figcaption></figcaption></figure>

Connect an Xbox controller to your system and drag and drop from **System Joysticks** to **Joystick\[0]**

Open up the Simulated mechanism with **NetworkTables -> SmartDashboard -> Elevator-> mechanism**

<figure><img src="../.gitbook/assets/Robot Simulation 8_30_2025 7_06_08 PM.png" alt=""><figcaption></figcaption></figure>

Resize **Elevator/mechanism** to your liking

Press **Teleoperated** in **Robot State** then you can use your controller like its controlling the real robot!

<figure><img src="../.gitbook/assets/Robot Simulation 2025-08-30 18-37-59.gif" alt=""><figcaption></figcaption></figure>

Congratulations on successfully programming your Elevator!! :tada::tada:
{% endstep %}
{% endstepper %}

{% hint style="info" %}
Examples can be found in the YAMS repository on GitHub.

[https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple\_elevator](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/simple_elevator)
{% endhint %}
