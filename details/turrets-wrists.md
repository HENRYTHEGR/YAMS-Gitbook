---
icon: castle
---

# Turrets/Wrists

## Why are Turrets and Wrists the same thing? Aren't they both Arms?

Turrets and Wrists are known as Pivots in YAMS. They are different from Arms as they are not impacted by gravity to such a significant extent.

## Basic Turret Config

From the YAMS examples we have the following as a basic config.

```java
TalonFXS                   turretMotor = new TalonFXS(1);
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(4, 0, 0)
      .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withContinuousWrapping(Degrees.of(0), Degrees.of(360))// Wrapping enabled bc the pivot can spin infinitely
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withMomentOfInertia(Meters.of(0.25), Pounds.of(4)); // MOI Calculation
SmartMotorController       motor       = new TalonFXSWrapper(turretMotor,
                                                             DCMotor.getNEO(1),
                                                             motorConfig);

PivotConfig                m_config         = new PivotConfig() 
      .withHardLimits(Degrees.of(0), Degrees.of(720)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("PivotExample", TelemetryVerbosity.HIGH); // Telemetry
Pivot                      m_pivot          = new Pivot(m_config, motor);
```

## Hard Limits

Hard limits are useful for simulation to emulate hard limits that may or may not be imposed on your robot. It is best to give this a wide range if your mechanism is undefined that way you can see exactly what might happen and notice "that could break the robot" beforehand.

```java
PivotConfig m_config = new PivotConfig()
      .withHardLimits(Degrees.of(0), Degrees.of(720)) // Hard limit bc wiring prevents infinite spinning
```

## MOI

The moment of inertia is only used in simulation to estimate the behavior of the Pivot. If you have your MOI similar to the real mechanism than the PID and FF might be similar, however this is unlikely and you will probably have to [set a sim only PID and FF](editor/simulation-only-pid-+-feedforward.md).

MOI is configured on `SmartMotorControllerConfig` via `.withMomentOfInertia(Distance, Mass)`.

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withMomentOfInertia(Meters.of(0.25), Pounds.of(4)); // MOI Calculation
```

## Code Reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/simple_robot/src/main/java/frc/robot/subsystems/TurretSubsystem.java" %}
