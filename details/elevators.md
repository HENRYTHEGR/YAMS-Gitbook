---
icon: elevator
---

# Elevators

## Basic Elevator Config

Given the [tutorial](../tutorials/elevator.md#create-a-smartmotorcontrollerconfig) and [how to find your mechanism circumference](../how-to/how-to-find-your-mechanism-circumference.md), we have a basic config below.

```java
SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Drum radius is required for elevators. Chain-driven: specify chain pitch and tooth count.
  .withDrumRadius(Inches.of(0.25), 22)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(4, 0, 0)
  .withTrapezoidalProfile(MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  .withSimClosedLoopController(4, 0, 0)
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withStartingPosition(Meters.of(0.5));// Starting height of the Elevator

// Vendor motor controller object
SparkMax spark = new SparkMax(4, MotorType.kBrushless);

// Create our SmartMotorController from our Spark and config with the NEO.
SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

ElevatorConfig elevconfig = new ElevatorConfig()
      .withHardLimits(Meters.of(0), Meters.of(3)) // Hard limits defined
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH) // Telemetry Name
      .withCarriageWeight(Pounds.of(16)); // Mass of the carriage

Elevator m_elevator = new Elevator(elevconfig, sparkSmartMotorController);
```

## Carriage Weight

The carriage weight defined via `.withCarriageWeight` is used for simulation of the elevator. The weight given should be the carriage weight. Any changes in carriage weight will require a retuning of the PID and FF, and could necessitate a [sim only PID and FF](editor/simulation-only-pid-+-feedforward.md) to maintain an easy comparison.

```java
ElevatorConfig elevconfig = new ElevatorConfig()
      .withCarriageWeight(Pounds.of(16)); // Weight of the carriage.
```

## Cascading Elevators

Cascading elevators need their gearings divided by the number of stages there are in the elevator.

```java
// Cascading Elevator with a 3:1 and 4:1 gearbox and 2 stages.
new MechanismGearing(GearBox.fromReductionStages(3, 4).div(2));
new MechanismGearing(GearBox.fromReductionStages(3, 4)).div(2);
```

## Linear Slides and Slanted Elevators

A linear slide is still an elevator! Just with an angle of 0deg! You can set the angle of an Elevator with `.withAngle`

```java
ElevatorConfig elevconfig = new ElevatorConfig()
      .withAngle(Degrees.of(0)); // Parallel to the ground, linear slide.
```

A linear slide is guaranteed to have a [different sim PID and FF](editor/simulation-only-pid-+-feedforward.md), so be sure to set them!

## Hard Limits and Soft Limits

The difference between hard limits and soft limits are ubiquitous among all mechanisms.

* Hard Limits are where physical stops that should not break are located along the mechanisms movement.
* Soft Limits are where the motor controller PID should limit itself to. It does not always limit itself within these limits which is why Hard Limits should also be used.

Hard limits are the simulation maximum boundaries.

```java
SmartMotorControllerConfig config = config.withSoftLimits(Meters.of(0), Meters.of(3)); // Soft limits
...
ElevatorConfig elevconfig = new ElevatorConfig()
      .withHardLimits(Meters.of(0), Meters.of(3)) // Hard limits defined 
```

## Code Reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/simple_elevator/java/frc/robot/subsystems/ElevatorSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/exponential_elevator/java/frc/robot/subsystems/ExponentiallyProfiledElevatorSubsystem.java" %}

