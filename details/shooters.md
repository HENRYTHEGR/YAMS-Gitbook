---
icon: crosshairs-simple
---

# Shooters

## Basic Shooter Config

Given the [tutorial](../tutorials/shooter-flywheels.md) we have a basic shooter config below.

```java
SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0)
  .withSimClosedLoopController(50, 0, 0)
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withMomentOfInertia(Inches.of(4), Pounds.of(1));

// Vendor motor controller object
SparkMax spark = new SparkMax(4, MotorType.kBrushless);

// Create our SmartMotorController from our Spark and config with the NEO.
SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

FlyWheelConfig shooterConfig = new FlyWheelConfig()
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig, sparkSmartMotorController);
```

## Code Reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/simple_shooter/java/frc/robot/subsystems/ShooterSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/hooded_shooter/java/frc/robot/subsystems/FlywheelSubsystem.java" %}
