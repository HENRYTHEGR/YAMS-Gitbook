---
icon: rocket
---

# Why YAMS?

## The problem with rolling your own subsystem

Writing a motor subsystem from scratch is repetitive and error-prone. A production-quality subsystem requires:

* Motor controller configuration (PID, feedforward, gearing, current limits, idle mode, ramp rates…)
* A physics simulation that actually matches your hardware
* Periodic telemetry publishing so you can see what's happening
* `Mechanism2d` visualizations for AdvantageScope
* Command factories that respect WPILib's requirement system
* Soft limits, hard limits, and temperature protection
* Separate sim-only tuning paths

Teams routinely spend days getting a single mechanism working in simulation, only to discover the real robot behaves differently because the sim wasn't accurate enough to tune against. YAMS eliminates that cycle.

## What YAMS replaces

A typical hand-rolled Arm subsystem is 150-300 lines of code: config, sim state, periodic updates, command factories, limit enforcement, and telemetry. The YAMS equivalent is roughly 15 lines — one `SmartMotorControllerConfig`, one `ArmConfig`, one `Arm`, and a couple of command wrappers.

```java
// Everything below replaces ~200 lines of boilerplate
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new SparkWrapper(new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1), motorConfig);

ArmConfig armConfig = new ArmConfig()
    .withLength(Feet.of(2))
    .withHardLimits(Degrees.of(-10), Degrees.of(120));

Arm arm = new Arm(armConfig, smc);
```

That's it. Simulation, telemetry, hard limits, and a physics-backed `Mechanism2d` all come for free.

## Physics simulation out of the box

Every YAMS mechanism type ships with a physics model:

* **Arm / Pivot** — `SingleJointedArmSim`, gravity compensation included
* **Elevator** — `ElevatorSim` with configurable carriage weight and angle
* **FlyWheel** — `FlywheelSim` with configurable moment of inertia

YAMS runs the simulation automatically when `RobotBase.isSimulation()` is true. Call `arm.simIterate()` in `simulationPeriodic()` and you're done — no additional sim wiring needed.

Because the simulation uses accurate physics, the PID and feedforward gains you tune in sim are a reliable starting point for real hardware. You may still need minor adjustments, but you won't be starting from zero on the field.

## TunerX in simulation (CTRE hardware)

For CTRE TalonFX and TalonFXS motors, YAMS feeds its physics results directly back through Phoenix's vendorsim layer via `getSimState()`. This means:

1. Run your robot code in simulation as normal
2. Open Phoenix Tuner X and connect to the simulation
3. Every TalonFX signal — position, velocity, voltage, current — reads exactly as it would from real hardware

You can watch closed-loop error, plot signal graphs, and verify your gains in TunerX's familiar interface without touching a physical robot. The same workflow you use to diagnose real hardware works identically in simulation.

## Live tuning without reflashing

`TelemetryVerbosity.HIGH` publishes all PID gains, feedforward constants, motion profile constraints, soft limits, and current limits as writable NetworkTable entries. The `Live Tuning` command (visible in Elastic or any NT-aware dashboard) applies those values to the running motor controller in real time.

Changing `kP` from 4 to 6 and seeing the mechanism respond takes seconds, not the minutes a redeploy would cost. See the [Live Tuning](integrations.md) page for a full field-by-field reference.

## Loop overruns

### roboRIO

The roboRIO 1 and roboRIO 2 are both CPU-constrained. Heavy subsystems with hand-rolled telemetry, simulation, and visualization code frequently push the 20 ms robot loop toward or past its budget, especially when multiple mechanisms run simultaneously. YAMS's telemetry and simulation paths are tightly optimized and only run the work that verbosity level requires — `LOW` publishes only the fields you actually need for competition, keeping loop time predictable.

### SystemCore

The SystemCore's baseline processing speed is significantly higher than either roboRIO generation. Loop overruns that were a concern on roboRIO hardware are not a practical issue on SystemCore even at `TelemetryVerbosity.HIGH` with multiple mechanisms active. Teams migrating from roboRIO to SystemCore can safely run full telemetry without worrying about budget.

## Safety by default

YAMS enforces mechanism safety automatically:

* **Hard limits** are enforced in simulation as physical stops — the mechanism cannot travel past them
* **Soft limits** are pushed to the motor controller so the closed-loop controller respects them independent of your command logic
* **Temperature cutoff** stops the motor if the controller overheats
* **`SmartMotorControllerConfigurationException`** is thrown at construction time if limits are misconfigured (e.g., lower limit greater than upper limit), so misconfigurations surface immediately in testing rather than during a match

## AdvantageKit integration

YAMS publishes telemetry in a layout that works naturally with AdvantageKit's IO layer. See the [AdvantageKit Integration](advantagekit-integration.md) page for patterns that combine YAMS's simulation fidelity with AdvantageKit's logging replay.
