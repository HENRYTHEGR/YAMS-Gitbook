---
description: >-
  System Identification (SysId) is the process of determining feedforward constants
  for your mechanism through statistical analysis of its inputs and outputs.
---

# System Identification (SysId)

## What is System Identification?

System Identification is the process of determining a mathematical model for the behavior of a system through statistical analysis of its inputs and outputs. This model describes how input voltage affects the way your measurements (typically encoder data) evolve over time.

The WPILib SysId tool takes a model and dataset and attempts to fit parameters which would make your model most closely match the dataset. Even an imperfect model is usually "good enough" to give accurate **feedforward control** of the mechanism, and even to estimate optimal gains for **feedback control**.

{% hint style="info" %}
For a complete understanding of SysId theory, visit the [WPILib System Identification Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html).
{% endhint %}

## The Feedforward Equations

Different mechanisms use different feedforward equations:

### Simple Motor (Flywheels, Turrets, Linear Sliders)

$$V = K_s \cdot sgn(\dot{d}) + K_v \cdot \dot{d} + K_a \cdot \ddot{d}$$

### Elevator

$$V = K_g + K_s \cdot sgn(\dot{d}) + K_v \cdot \dot{d} + K_a \cdot \ddot{d}$$

The constant term ($$K_g$$) accounts for gravity.

### Arm

$$V = K_g \cdot cos(\theta) + K_s \cdot sgn(\dot{\theta}) + K_v \cdot \dot{\theta} + K_a \cdot \ddot{\theta}$$

The cosine term ($$K_g$$) accounts for the effect of gravity at different arm angles.

## Types of Tests

A standard SysId routine consists of **two types of tests**, each run in **both directions** (forward and backward):

| Test Type | Description |
|-----------|-------------|
| **Quasistatic** | The mechanism is gradually sped up such that the voltage corresponding to acceleration is negligible (hence, "as if static"). This helps determine $$K_s$$ and $$K_v$$. |
| **Dynamic** | A constant "step voltage" is applied to the mechanism, so that the behavior while accelerating can be determined. This helps determine $$K_a$$. |

{% hint style="warning" %}
Running a "backwards" test directly after a "forwards" test is generally advisable as it will more or less reset the mechanism to its original position.
{% endhint %}

---

## Using YAMS Helper Functions

YAMS provides built-in `sysId()` methods on mechanism classes (`Arm`, `Elevator`, `Shooter`, etc.) that simplify the entire process.

{% hint style="danger" %}
This section covers SysId with Spark or Nova Motor Controllers.
The methodology for TalonFX and TalonFXS is slightly different.
{% endhint %}

### Prerequisites

Before running SysId:

* Soft and hard limits are already configured
* Gearing is correct
* Mechanism circumference is set (for linear mechanisms)

### Arm Example

```java
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ArmSubsystem extends SubsystemBase {

  private Arm arm; // Your configured YAMS Arm

  /**
   * Run SysId on the Arm.
   * 
   * Static test: runs the arm up and down with 7V step voltage.
   * Dynamic test: ramps from 0V to 7V at 2V per second.
   * Test duration: 4 seconds maximum.
   */
  public Command sysId() { 
    return arm.sysId(
      Volts.of(7),              // Step voltage for dynamic test
      Volts.of(2).per(Second),  // Ramp rate for quasistatic test
      Seconds.of(4)             // Maximum test duration
    );
  }
  
  // ... rest of subsystem
}
```

### Elevator Example

```java
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase {

  private Elevator elevator; // Your configured YAMS Elevator

  /**
   * Run SysId on the Elevator.
   */
  public Command sysId() { 
    return elevator.sysId(
      Volts.of(7),              // Step voltage
      Volts.of(2).per(Second),  // Ramp rate
      Seconds.of(4)             // Timeout
    );
  }
  
  // ... rest of subsystem
}
```

### Binding to Controller Buttons

```java
public class RobotContainer {

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final CommandXboxController controller = 
      new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    
    // Default command to hold position
    armSubsystem.setDefaultCommand(armSubsystem.set(0));
  }

  private void configureBindings() {
    // Run SysId while A is held - release to stop
    controller.a().whileTrue(armSubsystem.sysId());
    
    // Manual control for testing
    controller.x().whileTrue(armSubsystem.set(0.3));
    controller.y().whileTrue(armSubsystem.set(-0.3));
  }
}
```

{% hint style="info" %}
Use `whileTrue()` so you can release the button to immediately stop the test if the mechanism exceeds safe limits!
{% endhint %}

---

## Using WPILib SysIdRoutine Directly

If you need more control over the SysId process, or are working with a `SmartMotorController` directly without a mechanism class, you can use WPILib's `SysIdRoutine` directly.

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html" %}

### Creating the SysIdRoutine

The `SysIdRoutine` requires two objects:

1. **Config** - Test settings (ramp rate, step voltage, timeout)
2. **Mechanism** - Callbacks for driving motors and logging data

```java
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.motorcontrollers.SmartMotorController;

public class ManualSysIdSubsystem extends SubsystemBase {

  private SmartMotorController motor; // Your YAMS SmartMotorController

  // Mutable holders to avoid allocations during logging
  private final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  private final MutAngle m_position = new MutAngle(0, 0, Rotations);
  private final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 0, RotationsPerSecond);

  // Create the SysIdRoutine
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      // Config: ramp rate, step voltage, timeout
      new SysIdRoutine.Config(
          Volts.of(1).per(Seconds.of(1)),  // Quasistatic ramp rate (1 V/s)
          Volts.of(4),                      // Dynamic step voltage
          Seconds.of(10)                    // Timeout
      ),
      new SysIdRoutine.Mechanism(
          // Drive callback - applies voltage to motors
          (Voltage voltage) -> motor.setDutyCycle(
              voltage.in(Volts) / RobotController.getBatteryVoltage()
          ),
          // Log callback - records position, velocity, and voltage
          log -> {
            motor.updateTelemetry();
            motor.simIterate();
            log.motor("motor")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_position.mut_replace(motor.getMechanismPosition()))
                .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
          },
          this,           // Subsystem for requirements
          "MyMechanism"   // Name for logging
      )
  );

  /**
   * Returns the quasistatic test command.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns the dynamic test command.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
```

### Reusable SysIdRoutine Factory Method

You can create a static helper method to generate `SysIdRoutine` instances for any `SmartMotorController`:

```java
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import yams.motorcontrollers.SmartMotorController;

public class SystemIdentification {

  private static final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
  private static final MutAngle m_distance = new MutAngle(0, 0, Rotations);
  private static final MutAngularVelocity m_velocity = 
      new MutAngularVelocity(0, 0, RotationsPerSecond);

  /**
   * Creates a SysIdRoutine for a SmartMotorController.
   *
   * @param motor         The SmartMotorController to characterize
   * @param motorName     Name for logging (appears in SysId tool)
   * @param subsystemBase The subsystem that owns this motor
   * @return A configured SysIdRoutine
   */
  public static SysIdRoutine createRoutine(
      SmartMotorController motor, 
      String motorName, 
      SubsystemBase subsystemBase) {

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds.of(1)),  // Ramp rate: 1 V/s
            Volts.of(4),                      // Step voltage: 4 V
            Seconds.of(10)                    // Timeout: 10 s
        ),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> motor.setDutyCycle(
                voltage.in(Volts) / RobotController.getBatteryVoltage()),
            log -> {
              motor.updateTelemetry();
              motor.simIterate();
              log.motor(motorName)
                  .voltage(m_appliedVoltage.mut_replace(
                      motor.getDutyCycle() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(m_distance.mut_replace(motor.getMechanismPosition()))
                  .angularVelocity(m_velocity.mut_replace(motor.getMechanismVelocity()));
            },
            subsystemBase,
            motorName));
  }
}
```

Then use it in your subsystem:

```java
public class ShooterSubsystem extends SubsystemBase {
  
  private SmartMotorController shooterMotor;
  private final SysIdRoutine sysIdRoutine;
  
  public ShooterSubsystem() {
    // ... motor configuration ...
    sysIdRoutine = SystemIdentification.createRoutine(shooterMotor, "Shooter", this);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
```

### Binding All Four Tests

When using `SysIdRoutine` directly, you need to bind all four tests separately:

```java
private void configureBindings() {
  // Quasistatic tests
  controller.a().whileTrue(subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
  controller.b().whileTrue(subsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  
  // Dynamic tests  
  controller.x().whileTrue(subsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
  controller.y().whileTrue(subsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
}
```

{% hint style="warning" %}
Only log files with a **single routine** in them are usable for analysis. If you run a routine on one motor and then run a routine on another motor without extracting the log or power-cycling the roboRIO in between, analysis will fail.
{% endhint %}

### Running All Tests with One Command

You can combine all four tests into a single command sequence that runs automatically:

```java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public static Command getFullSysIdCommand(
    SysIdRoutine routine,
    double quasistaticTimeout,
    double dynamicTimeout,
    double delayBetweenTests) {
  
  return routine.quasistatic(SysIdRoutine.Direction.kForward)
      .withTimeout(quasistaticTimeout)
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse)
          .withTimeout(quasistaticTimeout))
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kForward)
          .withTimeout(dynamicTimeout))
      .andThen(Commands.waitSeconds(delayBetweenTests))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse)
          .withTimeout(dynamicTimeout));
}

// Usage:
controller.a().whileTrue(
    getFullSysIdCommand(sysIdRoutine, 4.0, 2.0, 1.0)
);
```

For mechanisms with limits (arms, elevators), you can add safety stops:

```java
public static Command getFullSysIdCommand(
    SysIdRoutine routine,
    double quasistaticTimeout,
    double dynamicTimeout,
    double delay,
    BooleanSupplier isAtMax,
    BooleanSupplier isAtMin,
    Runnable stopMotor) {
  
  return routine.quasistatic(SysIdRoutine.Direction.kForward)
      .until(isAtMax)
      .finallyDo(stopMotor)
      .withTimeout(quasistaticTimeout)
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse)
          .until(isAtMin)
          .finallyDo(stopMotor)
          .withTimeout(quasistaticTimeout))
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kForward)
          .until(isAtMax)
          .finallyDo(stopMotor)
          .withTimeout(dynamicTimeout))
      .andThen(Commands.waitSeconds(delay))
      .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse)
          .until(isAtMin)
          .finallyDo(stopMotor)
          .withTimeout(dynamicTimeout));
}
```

---

## Running the Tests

1. **Deploy your code** to the robot
2. **Ensure sufficient space** - at least 10-20 feet for drivetrains, adequate range of motion for arms/elevators
3. **Run all four tests** in sequence:
   - Quasistatic Forward
   - Quasistatic Reverse
   - Dynamic Forward
   - Dynamic Reverse
4. **Monitor carefully** - stop tests early if the mechanism exceeds safe limits

{% hint style="danger" %}
Watch out for your mechanism and stop the test early if it exceeds safe limits! The routine only creates voltage commands - it is up to you to set up hard or soft limits to prevent injury or damage.
{% endhint %}

---

## Analyzing Results

After running all tests:

1. **Retrieve the log file** from the roboRIO using the DataLogTool
2. **Open SysId** (included with WPILib)
3. **Load the log file** in the Log Loader pane
4. **Drag entries** to the Data Selector:
   - Find an entry containing "state" (type: string) → Test State slot
   - Find Position, Velocity, and Voltage entries → respective slots
5. **Select analysis type** (Simple, Elevator, or Arm)
6. **Click Load** and review diagnostics

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html" %}

### Applying the Results

Once you have your feedforward constants ($$K_s$$, $$K_v$$, $$K_a$$, and $$K_g$$ if applicable), apply them to your YAMS configuration:

```java
// For Arms
.withFeedforward(new ArmFeedforward(kS, kG, kV, kA))

// For Elevators  
.withFeedforward(new ElevatorFeedforward(kS, kG, kV, kA))

// For Simple Motors (Shooters, Turrets)
.withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))
```

{% hint style="success" %}
Remember to also set `.withSimFeedforward()` with the same or similar values for accurate simulation!
{% endhint %}

---

## YAMS vs Manual: When to Use Each

| Approach | Best For |
|----------|----------|
| **YAMS `mechanism.sysId()`** | Standard mechanisms (Arm, Elevator, Shooter) where you're already using YAMS mechanism classes. Simplest setup. |
| **WPILib `SysIdRoutine`** | Loosely coupled mechanisms, custom configurations, or when you need fine-grained control over the logging process. |

Both approaches produce compatible log files that work with the WPILib SysId analysis tool.

## Related Documentation

* [How to run SysId on an Arm](../how-to/how-to-run-sysid-on-a-arm.md)
* [How to run SysId on an Elevator](../how-to/how-to-run-sysid-on-a-elevator.md)
* [How do I control a Mechanism without a Mechanism Class?](../how-to/how-do-i-control-a-mechanism-without-a-mechanism-class.md)
* [WPILib SysId Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html)
